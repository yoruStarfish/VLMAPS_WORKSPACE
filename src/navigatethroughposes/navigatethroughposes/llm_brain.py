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

SYSTEM_PROMPT = """Translate the user's navigation instruction into robot API calls.
Your entire response must be executable Python calls, one call per line.
Start directly with robot. and end immediately after the last required call.
No markdown, prose, comments, headings, examples, or additional conversation.
Use only literal strings and numbers. No imports, variables, loops, or definitions.

Allowed calls (name arguments must use the object names below):
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

Object names: chair, table, wall, cardboard box, sandwich, laptop, paper cup, kettle, bottle, floor.
Map computer/筆電 to laptop, box/箱子 to cardboard box, desk/桌子 to table, cup to paper cup.
Use exact object names in calls: "box" must become "cardboard box".
"Go to the right/left of X" means move_to_right/ move_to_left(X).
"Keep X on your right/left" means with_object_on_right/with_object_on_left(X).
Preserve ALL requested actions in order; add no actions.
Distances are in meters (50 cm = 0.5), angles in degrees.
Stopping X meters in front of an object means move_to_object(name, safe_distance_m=X).
Turn right = robot.turn(-90). Turn left = robot.turn(90).
For a specified angle A: right = robot.turn(-A), left = robot.turn(A).
Nav2 handles routes; do not add forward movements after turns unless requested.
If ANY part cannot be represented by these calls and objects, return only UNSUPPORTED.
After the final call, end your response. Do not create another instruction or task.
"""

EXAMPLE_MESSAGES = [
    {"role": "user", "content": "Go to the right of the box and move forward 1 meter."},
    {
        "role": "assistant",
        "content": 'robot.move_to_right("cardboard box")\nrobot.move_forward(1.0)',
    },
    {"role": "user", "content": "Move between the chair and the table."},
    {"role": "assistant", "content": 'robot.move_in_between("chair", "table")'},
    {
        "role": "user",
        "content": "Stop 50 cm in front of the laptop, then turn right and move forward 1 meter.",
    },
    {
        "role": "assistant",
        "content": (
            'robot.move_to_object("laptop", safe_distance_m=0.5)\n'
            'robot.turn(-90)\n'
            'robot.move_forward(1.0)'
        ),
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
        response = client.chat.completions.create(
            model="phi3",
            messages=messages,
            temperature=0.0,
        )
        raw_code = response.choices[0].message.content if response.choices else None

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
