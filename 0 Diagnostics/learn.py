# F-string formatting (29 Jan; https://youtu.be/EoNOWVYKyo0)

import logging # TO LEARN

var: str = 'poop'
num: int = 1000000
dec: float = 1234.5678

print(f'{num:,}')
print(f'{num:>20}')
print(f'{num:$^20}')
print(f'{dec:,.2f}')

print(f"{num + dec = }")

def adder(*args:int) -> int:
    print(f"args:{args}")  # Prints the whole tuple
    print(*args) # Prints each element separated by spaces
    return sum(args)    # wait idk yet

adder(1,2,3,4, 5.6)
adder(5,6,8)

import glob
images = glob.glob("Diagnostics/calibration_images/*.jpg")  # Update with your image folder
print(f"Found {len(images)} images")


def upper_everything(elements: list[str]) -> list[str]:
    return [element.upper() for element in elements]

result = upper_everything(["hello", "what"])
print(result)