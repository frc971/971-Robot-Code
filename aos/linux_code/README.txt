see ../README.txt for stuff affecting crio and linux code

The folder is called linux_code because it mainly deals with code that uses the queue system, which only works under GNU/Linux for a variety of reasons, some fundamental (futexes) and some because nobody bothers to fix them.
The layout is designed with multiple linux boxes in mind.

The one that talks to the cRIO etc is called the prime. We have multiple explanations for that name:
  It is the primary controller.
  PRIME/Primary Robot Intelligent Management Entity
  Represents Optimus Prime, one of the good transformers who battle the bad ones that 254 names robots after.
  It is easy to type and doesn't conflict with anything else common for tab-completion.
  It's not hardware-specific.

[NOTES]
Any code should call aos::Init() (or aos::InitNRT() for processes that don't need to be realtime) before making any calls to any of the aos functions.
Making calls to any of the aos functions (including aos::Init()) from more than 1 thread per process is not supported, but using fork(2) after some aos functions have been called and then continuing to make aos function calls (without calling one of the exec(3) functions) in both processes is supported.
