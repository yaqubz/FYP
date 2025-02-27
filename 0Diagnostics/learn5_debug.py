def adder(*args:int) -> int:
    print(f"args:{args}")  # Prints the whole tuple
    print(*args) # Prints each element separated by spaces
    return sum(args)    # wait idk yet

a = adder(1,2,3,4, 5.6)
b = adder(5,6,8)

c = a + b
print(f"Test {c + a + b}")

def add_double(*args:int) -> int:
    return 2 * adder(*args)

c += add_double(1,2,3) + add_double(4,5,6)