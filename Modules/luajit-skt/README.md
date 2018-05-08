# luajit-skt
Send and receive data over UNIX sockets

## Installation

Ensure [LuaJIT 2.1](http://luajit.org/) is installed.

Install:
```sh
git clone https://github.com/StephenMcGill-TRI/luajit-skt.git
cd luajit-skt
luarocks make
```

## Operating System Settings

Constants were generated on Linux and macOS systems. However, there may be discrepenacies that arise. Generate new operating system specific flags via `./generate_constants.lua`. This requires a C compiler.
