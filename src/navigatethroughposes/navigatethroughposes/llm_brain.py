# llm_brain.py
import re
from openai import OpenAI

client = OpenAI(
    base_url='http://localhost:11434/v1',
    api_key='ollama', 
)

def get_code_from_llm(user_instruction):
    """接收人類語言，回傳過濾後的 Python 程式碼字串"""
    system_prompt = """
    You are a robot navigation assistant. 
    You have access to the following Python API:
    
    - go_to_object(object_name: str): move to the nearest object.
    
    Task: Translate the user's natural language instruction into Python code using ONLY the function provided. 
    Rule 1: Output ONLY valid Python code.
    Rule 2: Do not add any explanations or markdown formatting.
    """
    
    print("🧠 LLM 思考中...")
    response = client.chat.completions.create(
        model="phi3", 
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_instruction}
        ],
        temperature=0.0
    )
    
    raw_code = response.choices[0].message.content
    match = re.search(r'```(?:python)?\s*(.*?)\s*```', raw_code, re.DOTALL)
    if match:
        return match.group(1).strip()
    return raw_code.strip()



# ==========================================
# 主程式執行區
# ==========================================
# if __name__ == "__main__":
#     # 人類下達口語指令
#     user_command = "Move to the nearest sandwich."
#     print(f"👤 人類說: {user_command}\n")
    
#     # 步驟 A: 取得 LLM 產生的「程式碼字串」
#     code_string = ask_llm(user_command)
#     print(f"📝 LLM 產生的字串為: {code_string}\n")
    
#     # 步驟 B: 見證奇蹟的時刻！使用 exec() 執行該字串
#     print("⚡ 將字串交給系統執行...")
#     try:
#         exec(code_string) 
#     except Exception as e:
#         print(f"❌ 執行發生錯誤: {e}")