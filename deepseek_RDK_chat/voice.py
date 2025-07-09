#from bot import Bot
#gpt = Bot(model='gpt-4o-mini',tools=[],prompt="你是我的工作助手。",debug=True)
import os  
from dotenv import load_dotenv  
import io  
import azure.cognitiveservices.speech as speechsdk  
from openai import OpenAI
import time  
import datetime  
import threading  
import json, ast  
import re  # 新增正则表达式模块
import requests  
from io import BytesIO  
import tempfile  
#import pyaudio  
from tools import *
  
load_dotenv("1.env")  
client = OpenAI(api_key=os.environ["api_key"], base_url=os.environ["base_url"])
#client = OpenAI(api_key="1", base_url="http://localhost:11434/v1") 
Azure_speech_key = os.environ["Azure_speech_key"]  
Azure_speech_region = os.environ["Azure_speech_region"]  
Azure_speech_speaker = os.environ["Azure_speech_speaker"]  
WakeupWord = os.environ["WakeupWord"]  
WakeupModelFile = os.environ["WakeupModelFile"]  

messages = []  

# 设置Azure语音服务
speech_key = Azure_speech_key  
service_region = Azure_speech_region  
speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region)  
speech_config.speech_synthesis_language = "zh-CN"  
speech_config.speech_recognition_language = "zh-CN"  
lang = "zh-CN"  
speech_config.speech_synthesis_voice_name = Azure_speech_speaker  
speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config)  
connection = speechsdk.Connection.from_speech_synthesizer(speech_synthesizer)  
connection.open(True)  
model = speechsdk.KeywordRecognitionModel(WakeupModelFile)  
keyword = WakeupWord  
audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)  
auto_detect_source_language_config = speechsdk.languageconfig.AutoDetectSourceLanguageConfig(languages=["ja-JP", "zh-CN"])  
speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config,  
                                               auto_detect_source_language_config=auto_detect_source_language_config)  
unknownCount = 0  
sysmesg = {"role": "system", "content": os.environ["sysprompt_zh-CN"]}  
tts_sentence_end = [ ".", "!", "?", ";", "。", "！", "？", "；", "\n" ]

isListenning = False

# 新增：文本清洗函数
def clean_tts_text(text):
    """清除文本中不应被朗读的特殊符号"""
    if not isinstance(text, str):
        return str(text)
    
    # 移除Markdown格式
    text = re.sub(r'\*\*(.*?)\*\*', r'\1', text)  # **强调** → 强调
    text = re.sub(r'_(.*?)_', r'\1', text)         # _斜体_ → 斜体
    text = re.sub(r'`(.*?)`', r'\1', text)         # `代码` → 代码
    
    # 替换特殊符号
    symbol_map = {
        r'\*': ' ',
        r'_': ' ',
        r'#': '，',
        r'\[': ' ',
        r'\]': ' ',
        r'\{': ' ',
        r'\}': ' ',
        r'\(': ' ',
        r'\)': ' ',
        r'<': ' ',
        r'>': ' ',
        r'\|': '，',
        r'~': ' ',
        r'\^': ' ',
        r'\\': ' ',
        r'/': ' ',
        r'@': ' ',
        r'\+': '加',
        r'=': '等于'
    }
    
    for pattern, replacement in symbol_map.items():
        text = re.sub(pattern, replacement, text)
    
    # 处理数字（简单中文读法）
    def num_to_chinese(match):
        num = match.group(0)
        if num.isdigit():
            # 简单转换，复杂情况可以使用第三方库
            chinese_digits = ['零', '一', '二', '三', '四', '五', '六', '七', '八', '九']
            return ''.join(chinese_digits[int(d)] for d in num)
        return num
    
    text = re.sub(r'\d+', num_to_chinese, text)
    
    # 合并多余空格
    text = re.sub(r'\s+', ' ', text).strip()
    
    return text

def display_text(s):
    print(s)
    
def speech_to_text():  
    global unknownCount  
    global lang, isListenning  
    print("请说话...")  
    try:
        result = speech_recognizer.recognize_once_async().get()  
        if result.reason == speechsdk.ResultReason.RecognizedSpeech:  
            unknownCount = 0  
            isListenning = False
            return result.text  
        elif result.reason == speechsdk.ResultReason.NoMatch:  
            isListenning = False
            unknownCount += 1  
            error = os.environ.get(f"sorry_{lang}", "抱歉，我没听清楚")  
            text_to_speech(error)  
            return '...'  
        elif result.reason == speechsdk.ResultReason.Canceled:  
            isListenning = False
            return "语音识别已取消"
    except Exception as e:
        print(f"语音识别错误: {e}")
        return "语音识别错误"
    

def getVoiceSpeed():  
    return 17  
  
def text_to_speech(text, _lang=None):  
    global lang  
    try:  
        result = buildSpeech(text).get()  
        if result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:  
            print("文本转语音成功")  
            return "成功"  
        else:  
            print(f"语音合成错误: {result}")  
            return "失败"  
    except Exception as ex:  
        print(f"语音合成异常: {ex}")  
        return "错误"  
        
def buildSpeech(text, _lang=None):
    # 清洗文本
    cleaned_text = clean_tts_text(text)
    
    voice_lang = lang  
    voice_name = Azure_speech_speaker
    ssml_text = f'''  
        <speak xmlns="http://www.w3.org/2001/10/synthesis" 
               xmlns:mstts="http://www.w3.org/2001/mstts" 
               version="1.0" 
               xml:lang="{lang}">
            <voice name="{voice_name}">
                <lang xml:lang="{voice_lang}">
                    <prosody rate="{getVoiceSpeed()}%">
                        {cleaned_text}
                    </prosody>
                </lang>
            </voice>
        </speak>  
    '''  
    print(f"[语音合成] {cleaned_text}")  
    return speech_synthesizer.speak_ssml_async(ssml_text)
  
def generate_text(prompt):  
    global messages  
    messages.append({"role": "user", "content": prompt})  
    tools = getTools()  
    collected_messages = []
    last_tts_request = None

    result = ''
    function_list = []
    
    try:
        response_gen = client.chat.completions.create(
            model="deepseek-chat",
            messages=[sysmesg] + messages[-20:],
            tools=tools,
            stream=False
        )
        response_message = response_gen.choices[0].message
        
        # 处理工具调用
        if response_message.tool_calls:
            tool_calls = response_message.tool_calls
            function_results = []
            
            for tool_call in tool_calls:
                function_name = tool_call.function.name
                function_args = json.loads(tool_call.function.arguments)
                function_to_call = globals().get(function_name)
                
                if not function_to_call:
                    error_msg = f"⚠️ 函数不存在: {function_name}"
                    print(error_msg)
                    function_results.append(error_msg)
                    continue
                    
                try:
                    print(f"🛠️ 调用函数: {function_name}({function_args})")
                    result = function_to_call(**function_args)
                    function_results.append(str(result))
                    print(f"✅ 函数结果: {result}")
                except Exception as e:
                    error_msg = f"⚠️ {function_name}执行错误: {str(e)}"
                    print(error_msg)
                    function_results.append(error_msg)
            
            # 合并结果并添加到对话历史
            combined_result = "\n".join(function_results)
            messages.append({
                "role": "assistant", 
                "content": f"【工具调用结果】\n{combined_result}"
            })
            
            # 语音播报结果
            last_tts_request = buildSpeech(combined_result)
            result = combined_result
        
        # 处理普通回复
        elif response_message.content and response_message.content != '':
            result = response_message.content
            messages.append({"role": "assistant", "content": result})
            last_tts_request = buildSpeech(result)
            
    except Exception as e:
        error_msg = f"⚠️ 生成文本时出错: {str(e)}"
        print(error_msg)
        result = error_msg
        last_tts_request = buildSpeech("抱歉，处理请求时出错了")
    
    # 等待语音合成完成
    if last_tts_request:
        try:
            last_tts_request.get()
        except Exception as e:
            print(f"⚠️ 语音合成等待错误: {str(e)}")
    
    return result

def Get_Chat_Deployment():  
    return os.environ.get("Azure_OPENAI_Chat_API_Deployment", "default-deployment")  

def recognized_cb(evt):  
    result = evt.result  
    if result.reason == speechsdk.ResultReason.RecognizedKeyword:  
        print("识别到关键词: {}".format(result.text))  
    global done  
    done = True  

def canceled_cb(evt):  
    result = evt.result  
    if result.reason == speechsdk.ResultReason.Canceled:  
        print('已取消: {}'.format(result.cancellation_details.reason))  
    global done  
    done = True  
   
def start_recognition():  
    global unknownCount, isListenning
    print("语音助手启动中...")
    
    # 初始问候
    first = os.environ.get(f"welcome_{lang}", "您好，我是语音助手")
    display_text(first) 
    text_to_speech(first)
    
    while True:  
        keyword_recognizer = speechsdk.KeywordRecognizer()  
        keyword_recognizer.recognized.connect(recognized_cb)  
        keyword_recognizer.canceled.connect(canceled_cb) 
        
        # 播放唤醒提示
        wake_prompt = os.environ.get(f"listen_{lang}", "等待唤醒词")
        display_text(wake_prompt)
        text_to_speech(wake_prompt)
        
        isListenning = True
        result_future = keyword_recognizer.recognize_once_async(model)  
        while True:  
            result = result_future.get()
            if result.reason == speechsdk.ResultReason.RecognizedKeyword:
                print("✅ 识别到唤醒词")
                isListenning = False
                if getPlayerStatus() == 'playing':
                    pauseplay()  # 被唤醒后暂停音乐
                break
            time.sleep(0.1)  
            
        # 唤醒后响应
        wake_response = os.environ.get(f"wake_{lang}", "我在听，请讲")
        display_text(wake_response)  
        text_to_speech(wake_response)  
          
        # 对话循环
        while unknownCount < 2:
            isListenning = True
            user_input = speech_to_text()
            
            # 处理识别问题
            if user_input == '...':
                continue
                
            print(f"用户: {user_input}")  
            display_text(f"用户: {user_input}")  
            
            # 生成响应
            response = generate_text(user_input)
            
            # 检查是否应结束对话
            if response.strip().lower() in ["退出", "结束", "再见"]:
                break
                
            # 检查音乐状态
            if getPlayerStatus() == 'playing':
                break
          
        # 结束对话
        bye_text = os.environ.get(f"bye_{lang}", "再见")  
        display_text(bye_text) 
        text_to_speech(bye_text)  
        
        unknownCount = 0  
        time.sleep(0.1)  

if __name__ == "__main__":
    try:
        start_recognition()
    except KeyboardInterrupt:
        print("\n程序已退出")
    except Exception as e:
        print(f"⚠️ 主程序错误: {str(e)}")
        text_to_speech("抱歉，系统遇到严重错误")