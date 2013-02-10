see ../README.txt for stuff affecting crio and atom code

[NOTES]
Any code should call aos::Init() (or aos::InitNRT() for processes that don't need to be realtime) before making any calls to any of the aos functions.
Making calls to any of the aos functions (including aos::Init()) from more than 1 thread per process is not supported, but using fork(2) after some aos functions have been called and then continuing to make aos function calls (without calling one of the exec(3) functions) in both processes is supported.

