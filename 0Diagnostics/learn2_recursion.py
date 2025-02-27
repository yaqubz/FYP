def factorial(n):
    """
    Calculate factorial using recursion.
    Example: factorial(5) = 5 * 4 * 3 * 2 * 1 = 120
    """
    # Base case: factorial of 0 or 1 is 1
    if n <= 1:
        return 1
    # Recursive case: n! = n * (n-1)!
    return n * factorial(n - 1)

def fibonacci(n):
    """
    Calculate the nth Fibonacci number using recursion.
    Example: fibonacci(6) = 8 (sequence: 1,1,2,3,5,8,...)
    """
    # Base cases: first two numbers are 1
    if n <= 2:
        return 1
    # Recursive case: fib(n) = fib(n-1) + fib(n-2)
    return fibonacci(n - 1) + fibonacci(n - 2)

def traverse_directory(path, indent=0):
    """
    Recursively traverse and print a directory structure.
    Practical example of using recursion for tree-like structures.
    """
    import os
    
    # Print current directory/file
    print(' ' * indent + '|-', os.path.basename(path))
    
    # Base case: if it's a file, stop recursion
    if not os.path.isdir(path):
        return
        
    # Recursive case: if it's a directory, traverse its contents
    try:
        for item in os.listdir(path):
            item_path = os.path.join(path, item)
            traverse_directory(item_path, indent + 2)
    except PermissionError:
        print(' ' * (indent + 2) + '|- [Permission Denied]')

# Example usage
if __name__ == "__main__":
    print("Factorial Examples:")
    for i in range(6):
        print(f"factorial({i}) = {factorial(i)}")
    
    print("\nFibonacci Examples:")
    for i in range(1, 8):
        print(f"fibonacci({i}) = {fibonacci(i)}")
    
    print("\nDirectory Structure Example:")
    traverse_directory(".", indent=0)  # Traverse current directory