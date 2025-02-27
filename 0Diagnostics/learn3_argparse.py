import argparse

parser = argparse.ArgumentParser()

SIMULATE = True  # Default value

parser.add_argument('-sim', '--simulate', type=int, choices=[0, 1], help="Set simulation mode: 1 for True, 0 for False")

args = parser.parse_args()

if args.simulate is not None:
    print("HERE")
    print(args.simulate)
    SIMULATE = bool(args.simulate)  # Convert 0/1 to False/True

print(SIMULATE)

## Arguments with flags are optional (TBC)
# parser.add_argument('greeting', help="the greeting message displayed") # this is a required argument
# parser.add_argument('-n', '--numbers', type=float, nargs=2, help="two numbers to be added")
# parser.add_argument('-s', '--sum', type=float, nargs='*', help="any numbers to be added")

## Argument type cannot be bool - use action='store_true' instead
# parser.add_argument('-sim', '--simulate', action='store_true', help="Enable simulation mode")
# parser.add_argument('-nosim', '--no-simulate', action='store_false', dest='simulate', help="Disable simulation mode")



# print(args)
# print(args.greeting)
# print(args.numbers)
# print(args.sum)

# if args.numbers is not None:
#     print(args.numbers[0] + args.numbers[1])

# if args.sum is not None:
#     print(sum(args.sum))

# if args.simulate is not None:
#     print(args.simulate)
#     SIMULATE = args.simulate

"""
python3 .\Diagnostics\learn3_argparse.py hello -n 2 3 -s 2 3 4 5 
"""

