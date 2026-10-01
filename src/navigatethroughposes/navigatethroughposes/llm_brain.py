# # llm_brain.py
# import re
# from openai import OpenAI

# client = OpenAI(
#     base_url='http://localhost:11434/v1',
#     api_key='ollama', 
# )

# def get_code_from_llm(user_instruction):
#     """接收人類語言，回傳過濾後的 Python 程式碼字串"""
#     system_prompt = """
#     You are a robot navigation assistant. 
#     You have access to the following Python API:
    
#     - go_to_object(object_name: str): move to the nearest object.
    
#     Task: Translate the user's natural language instruction into Python code using ONLY the function provided. 
#     Rule 1: Output ONLY valid Python code.
#     Rule 2: Do not add any explanations or markdown formatting.
#     """
    
#     print("🧠 LLM 思考中...")
#     response = client.chat.completions.create(
#         model="phi3", 
#         messages=[
#             {"role": "system", "content": system_prompt},
#             {"role": "user", "content": user_instruction}
#         ],
#         temperature=0.0
#     )
    
#     raw_code = response.choices[0].message.content
#     match = re.search(r'```(?:python)?\s*(.*?)\s*```', raw_code, re.DOTALL)
#     if match:
#         return match.group(1).strip()
#     return raw_code.strip()


# llm_brain.py
import re
from openai import OpenAI

client = OpenAI(
    base_url='http://localhost:11434/v1',
    api_key='ollama', 
)

def get_code_from_llm(user_instruction):
    """接收人類語言，回傳過濾後的 Python 程式碼字串"""
    system_prompt = """You are a robot navigation assistant.
Translate the user's natural language instruction into Python code using ONLY the `robot` API provided below.

Available Python API:
- robot.move_to_object(name: str): Move to the nearest specified object.
- robot.move_to_left(name: str): Move to the left of the specified object.
- robot.move_to_right(name: str): Move to the right of the specified object.
- robot.move_in_between(name_a: str, name_b: str): Move to the middle between two objects.
- robot.move_north(name: str): Move to the north of the object.
- robot.move_south(name: str): Move to the south of the object.
- robot.move_east(name: str): Move to the east of the object.
- robot.move_west(name: str): Move to the west of the object.
- robot.face(name: str): Turn in place to face the specified object.
- robot.with_object_on_left(name: str): Turn so the object is on the robot's left side.
- robot.with_object_on_right(name: str): Turn so the object is on the robot's right side.
- robot.move_forward(meters: float): Move forward by the given meters.
- robot.turn(angle_deg: float): Turn in place by the given degrees (positive for right, negative for left).

Valid object names (you MUST map user terms to these exact strings):
"chair", "table", "wall", "cardboard box", "sandwich", "laptop", "paper cup", "kettle", "bottle", "floor"

Rules:
1. Output ONLY valid executable Python code.
2. Do NOT include any explanations, comments, or markdown formatting.
3. Map synonyms to valid object names (e.g., "cup" -> "paper cup", "desk" -> "table", "box" -> "cardboard box", "computer" -> "laptop").

Examples:
User: go to the laptop
robot.move_to_object("laptop")

User: move between the chair and the table, then look at the cup
robot.move_in_between("chair", "table")
robot.face("paper cup")

User: go to the right of the box and move forward 1 meter
robot.move_to_right("cardboard box")
robot.move_forward(1.0)
"""
    
    print("LLM 思考中...", flush=True)
    response = client.chat.completions.create(
        model="phi3", 
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_instruction}
        ],
        temperature=0.0
    )
    
    raw_code = response.choices[0].message.content
    
    # 若模型仍附帶了 markdown 標籤，將其剝除
    match = re.search(r'```(?:python)?\s*(.*?)\s*```', raw_code, re.DOTALL)
    if match:
        code_lines = match.group(1).strip().splitlines()
    else:
        code_lines = raw_code.strip().splitlines()
        
    # 過濾掉模型可能不小心吐出的非程式碼文字（只保留以 robot. 或 go_to_object 開頭的有效行）
    valid_lines = [
        line.strip() for line in code_lines 
        if line.strip().startswith("robot.") or line.strip().startswith("go_to_object")
    ]
    
    if valid_lines:
        return "\n".join(valid_lines)
    return raw_code.strip()


