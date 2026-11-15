"""Translate navigation instructions into validated robot API calls."""
import ast
import math
import re
from openai import OpenAI

client = OpenAI(base_url="http://localhost:11434/v1", api_key="ollama")

OBJECT_NAMES = {
    "chair", "table", "wall", "cardboard box", "sandwich", "laptop",
    "paper cup", "kettle", "bottle", "floor",
}

API_PARAMETERS = {
    "move_to_object": ("name", "safe_distance_m"),
    "move_in_between": ("name_a", "name_b"),
    "turn": ("angle_deg",),
    "move_forward": ("meters",),
    **{name: ("name",) for name in (
        "move_to_left", "move_to_right", "move_north", "move_south",
        "move_east", "move_west", "face", "with_object_on_left",
        "with_object_on_right",
    )},
}



SYSTEM_PROMPT = """You are a robot navigation command translator.
Your ONLY task is to convert user instructions into robot API calls.

OUTPUT FORMAT:
- Output ONLY executable Python robot API calls, one per line.
- Every line MUST start with "robot.".
- No explanations, reasoning, comments, markdown, or extra text.
- Do NOT discuss ambiguity or suggest alternative interpretations.
- Do NOT generate User: or Assistant: messages.
- Stop immediately after the final required API call.
- Use only literal strings and numbers.
- No imports, variables, loops, or function definitions.

AVAILABLE APIs:
robot.move_to_object(name, safe_distance_m=0.5)
robot.move_in_between(name_a, name_b)
robot.move_to_left(name)
robot.move_to_right(name)
robot.move_north(name)
robot.move_south(name)
robot.move_east(name)
robot.move_west(name)
robot.face(name)
robot.with_object_on_left(name)
robot.with_object_on_right(name)
robot.move_forward(meters)
robot.turn(angle_deg)

VALID OBJECT NAMES:
chair, table, wall, cardboard box, sandwich,
laptop, paper cup, kettle, bottle, floor

OBJECT ALIASES:
box/箱子 -> cardboard box
computer/筆電 -> laptop
desk/桌子 -> table
cup -> paper cup

NAVIGATION RULES:
1. "Go to the right of X" -> robot.move_to_right("X")
2. "Go to the left of X" -> robot.move_to_left("X")
3. "Keep X on your right" -> robot.with_object_on_right("X")
4. "Keep X on your left" -> robot.with_object_on_left("X")
5. "Face X" -> robot.face("X")
6. "Move between X and Y" -> robot.move_in_between("X", "Y")
7. "Stop D meters in front of X" ->
   robot.move_to_object("X", safe_distance_m=D)
8. "Move forward D meters" -> robot.move_forward(D)
9. "Turn right" -> robot.turn(-90)
10. "Turn left" -> robot.turn(90)
11. For specified turns, right is negative and left is positive.
12. Convert centimeters to meters.

CRITICAL RULES:
- "Go to the right of X" is a COMPLETE navigation action.
- It means moving to a position on the right side of X.
- It does NOT mean turning right.
- NEVER add robot.turn() or robot.move_forward() unless explicitly requested.
- If an instruction matches an available API, output that API immediately.
- Preserve ALL requested actions in their original order.
- NEVER invent additional movements, distances, or rotations.
- Nav2 handles navigation routes automatically.
- Do not add forward movement after turning unless requested.
- If ANY requested action or object is unsupported, output ONLY:
UNSUPPORTED

FINAL REQUIREMENT:
Generate the minimum number of API calls necessary to represent
the user's complete instruction.
Do not explain your answer.
Do not continue the conversation.
Stop generating immediately after the last API call.
"""


EXAMPLE_MESSAGES = [
    {
        "role": "user",
        "content": "Go to the right of the bottle."
    },
    {
        "role": "assistant",
        "content": 'robot.move_to_right("bottle")'
    },
    {
        "role": "user",
        "content": "Go to the right of the box and move forward 1 meter."
    },
    {
        "role": "assistant",
        "content": (
            'robot.move_to_right("cardboard box")\n'
            'robot.move_forward(1.0)'
        )
    },
    {
        "role": "user",
        "content": "Move between the chair and the table."
    },
    {
        "role": "assistant",
        "content": 'robot.move_in_between("chair", "table")'
    },
    {
        "role": "user",
        "content": (
            "Stop 50 cm in front of the laptop, "
            "then turn right and move forward 1 meter."
        )
    },
    {
        "role": "assistant",
        "content": (
            'robot.move_to_object("laptop", safe_distance_m=0.5)\n'
            'robot.turn(-90)\n'
            'robot.move_forward(1.0)'
        )
    },
]



def validate_robot_code(raw_code: str) -> str:
    """Reject the entire response if any call is invalid; never drop steps."""
    if not isinstance(raw_code, str) or not raw_code.strip():
        raise ValueError("No navigation command returned by the LLM.")
    
    code = raw_code.strip()
    match = re.fullmatch(r"```(?:python)?\s*\n?(.*?)\n?```", code, re.DOTALL)
    if match:
        code = match.group(1).strip()
        
    if code == "UNSUPPORTED":
        raise ValueError("This command is unsupported by the current robot API or object list.")
        
    try:
        tree = ast.parse(code)
    except SyntaxError as exc:
        raise ValueError("Invalid Python syntax returned by the LLM.") from exc
        
    if not tree.body:
        raise ValueError("Navigation command is empty.")
        
    for statement in tree.body:
        if not isinstance(statement, ast.Expr) or not isinstance(statement.value, ast.Call):
            raise ValueError("Only robot API calls are allowed.")
            
        call = statement.value
        func = call.func
        
        if not (isinstance(func, ast.Attribute) and isinstance(func.value, ast.Name)
                and func.value.id == "robot" and func.attr in API_PARAMETERS):
            raise ValueError("Contains unsupported API calls.")
            
        parameters = API_PARAMETERS[func.attr]
        
        if len(call.args) > len(parameters):
            raise ValueError("Too many API parameters.")
            
        bound = dict(zip(parameters, call.args))
        
        for keyword in call.keywords:
            if keyword.arg not in parameters or keyword.arg in bound:
                raise ValueError("Unknown or duplicate API parameter.")
            bound[keyword.arg] = keyword.value
            
        required = parameters[:1] if func.attr == "move_to_object" else parameters
        if any(name not in bound for name in required):
            raise ValueError("Missing required API parameters.")
            
        for name, node in bound.items():
            if name in ("name", "name_a", "name_b"):
                if not (isinstance(node, ast.Constant) and isinstance(node.value, str)
                        and node.value in OBJECT_NAMES):
                    raise ValueError(f"Object name '{node.value if hasattr(node, 'value') else node}' is not in the supported list.")
            else:
                number_node = node
                sign = 1
                if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
                    number_node = node.operand
                    sign = -1 if isinstance(node.op, ast.USub) else 1
                    
                if not (isinstance(number_node, ast.Constant) and type(number_node.value) in (int, float)):
                    raise ValueError("Distances and angles must be numeric constants.")
                    
                value = sign * number_node.value
                if not math.isfinite(value) or (name != "angle_deg" and value <= 0):
                    raise ValueError("Distances must be greater than zero; angles and distances must be finite numbers.")
                    
    return ast.unparse(tree)


def get_code_from_llm(user_instruction):
    """Return validated calls; raise ValueError instead of returning partial code."""
    if not isinstance(user_instruction, str) or not user_instruction.strip():
        raise ValueError("Please provide a non-empty navigation instruction.")
        
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        *EXAMPLE_MESSAGES,
        {"role": "user", "content": user_instruction.strip()},
    ]

    for attempt in range(2):
        print(f"LLM is thinking... ({attempt + 1}/2)", flush=True)
        import time

        start = time.perf_counter()

        stream = client.chat.completions.create(
            model="phi3",
            messages=messages,
            temperature=0.0,
            max_tokens=128,
            stream=True,
            stop=[
                "\n\n",
                "\nPlease",
                "\nExplanation:",
                "\nUser:",
                "\nAssistant:",
            ],
        )

        chunks = []

        for chunk in stream:
            if not chunk.choices:
                continue

            content = chunk.choices[0].delta.content

            if content:
                print(content, end="", flush=True)
                chunks.append(content)

        raw_code = "".join(chunks)

        print(f"\nLLM inference time: {time.perf_counter() - start:.2f}s")
        print(f"Raw code: {raw_code!r}")
        raw_code = "".join(chunks)

        print(f"\nLLM inference time: {time.perf_counter() - start:.2f}s")
        print(f"Raw code: {raw_code!r}")

        try:
            return validate_robot_code(raw_code)
        except ValueError as exc:
            print(f"LLM 回覆驗證失敗: {exc}", flush=True)
            print(f"原始回覆: {raw_code!r}", flush=True)
            if attempt == 1 or (isinstance(raw_code, str) and raw_code.strip() == "UNSUPPORTED"):
                raise
            messages.extend([
                {"role": "assistant", "content": raw_code or ""},
                {
                    "role": "user",
                    "content": (
                        f"Your previous response failed validation: {exc}\n"
                        f"Original instruction: {user_instruction.strip()}\n"
                        "Return only valid robot API calls for that instruction. "
                        "Do not include User:, Assistant:, examples, markdown, or explanations. "
                        "If unsupported, return UNSUPPORTED."
                    ),
                },
            ])
