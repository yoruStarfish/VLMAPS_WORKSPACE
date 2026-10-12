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

SYSTEM_PROMPT = """You are a robot navigation assistant.
Translate the instruction into Python calls using ONLY the robot API below.

Available API:
- robot.move_to_object(name: str, safe_distance_m: float = 0.5)
- robot.move_to_left(name: str)
- robot.move_to_right(name: str)
- robot.move_in_between(name_a: str, name_b: str)
- robot.move_north(name: str)
- robot.move_south(name: str)
- robot.move_east(name: str)
- robot.move_west(name: str)
- robot.face(name: str)
- robot.with_object_on_left(name: str)
- robot.with_object_on_right(name: str)
- robot.move_forward(meters: float)
- robot.turn(angle_deg: float): Positive is LEFT/counterclockwise; negative is RIGHT/clockwise.

Valid object names:
chair, table, wall, cardboard box, sandwich, laptop, paper cup, kettle, bottle, floor

Rules:
1. Output ONLY Python calls, one per line. No comments, markdown, or explanations.
2. Use literal strings and numbers only. No variables or loops.
3. Preserve every action in order, and preserve all specified distances.
4. Convert distances to meters and angles to degrees (e.g., 50 cm = 0.5).
5. Map synonyms or cross-language terms to exact valid object names (e.g., "computer"/筆電 -> "laptop", "box"/箱子 -> "cardboard box", "desk"/桌子 -> "table").
6. If the user specifies stopping X meters in front of an object, use `robot.move_to_object(name, safe_distance_m=X)`.
7. If no turn angle is given, assume a right turn is -90 and a left turn is 90.
8. Do not invent extra movements. Nav2 plans the route to each object; a preceding turn does not require the next route to be a straight line.
9. If an instruction cannot be expressed using these APIs and object names, output UNSUPPORTED only.

Examples:
User: Go to 1 meter in front of the laptop, turn right, and stop 0.5 meters in front of the kettle.
robot.move_to_object("laptop", safe_distance_m=1.0)
robot.turn(-90)
robot.move_to_object("kettle", safe_distance_m=0.5)

User: Move 50 cm in front of the kettle, then turn left 45 degrees.
robot.move_to_object("kettle", safe_distance_m=0.5)
robot.turn(45)

User: Move between the chair and the table, then look at the cup.
robot.move_in_between("chair", "table")
robot.face("paper cup")

User: Go to the right of the box and move forward 1 meter.
robot.move_to_right("cardboard box")
robot.move_forward(1.0)
"""

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
        
    print("LLM is thinking...", flush=True)
    
    response = client.chat.completions.create(
        model="phi3",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_instruction},
        ],
        temperature=0.0,
    )
    
    return validate_robot_code(response.choices[0].message.content)