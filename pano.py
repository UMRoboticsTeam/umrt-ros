from sys import argv
from os import system

def main() -> None:
    for file in argv[1:]:
        print(f"Using File {file}")
    system('convert +append ' + ' '.join(argv[1:]) + ' output.png')


if __name__ == '__main__':
    main()
