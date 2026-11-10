from math import sqrt


def require_dict(value, field):
    if not isinstance(value, dict):
        raise ValueError(f"'{field}' must be an object.")
    return value


def require_string(value, field):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"'{field}' must be a non-empty string.")
    return value


def require_bool(value, field):
    if not isinstance(value, bool):
        raise ValueError(f"'{field}' must be a boolean.")
    return value


def require_integer(value, field, minimum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"'{field}' must be an integer.")
    if minimum is not None and value < minimum:
        raise ValueError(f"'{field}' must be at least {minimum}.")
    return value


def require_number(value, field, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"'{field}' must be numeric.")
    number = float(value)
    if minimum is not None and number < minimum:
        raise ValueError(f"'{field}' must be at least {minimum}.")
    if maximum is not None and number > maximum:
        raise ValueError(f"'{field}' must be at most {maximum}.")
    return number


def require_vector(value, field, length, positive=False):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"'{field}' must contain exactly {length} numbers.")
    result = [require_number(item, f"{field}[{index}]") for index, item in enumerate(value)]
    if positive and any(item <= 0.0 for item in result):
        raise ValueError(f"All values in '{field}' must be positive.")
    return result


def normalize_quaternion(value, field="orientation"):
    quaternion = require_vector(value, field, 4)
    length = sqrt(sum(component * component for component in quaternion))
    if length < 1e-9:
        raise ValueError(f"'{field}' cannot be a zero quaternion.")
    return [component / length for component in quaternion]


def subtract(left, right):
    return [left[index] - right[index] for index in range(3)]


def cross(left, right):
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def normalize_vector(vector):
    length = sqrt(sum(value * value for value in vector))
    if length < 1e-9:
        raise ValueError("Viewpoint position and look_at point must differ.")
    return [value / length for value in vector]


def quaternion_from_rotation_matrix(matrix):
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        scale = sqrt(trace + 1.0) * 2.0
        return [
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            0.25 * scale,
        ]
    if m00 > m11 and m00 > m22:
        scale = sqrt(1.0 + m00 - m11 - m22) * 2.0
        return [
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
            (m21 - m12) / scale,
        ]
    if m11 > m22:
        scale = sqrt(1.0 + m11 - m00 - m22) * 2.0
        return [
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
            (m02 - m20) / scale,
        ]

    scale = sqrt(1.0 + m22 - m00 - m11) * 2.0
    return [
        (m02 + m20) / scale,
        (m12 + m21) / scale,
        0.25 * scale,
        (m10 - m01) / scale,
    ]


def viewpoint_quaternion(position, look_at):
    forward = normalize_vector(subtract(look_at, position))
    reference_up = [0.0, 0.0, 1.0]
    if abs(sum(a * b for a, b in zip(forward, reference_up))) > 0.98:
        reference_up = [0.0, 1.0, 0.0]

    right = normalize_vector(cross(reference_up, forward))
    up = cross(forward, right)
    rotation = [
        [right[0], up[0], forward[0]],
        [right[1], up[1], forward[1]],
        [right[2], up[2], forward[2]],
    ]
    return quaternion_from_rotation_matrix(rotation)

