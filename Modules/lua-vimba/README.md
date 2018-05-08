# lua-vimba
Interface the [VIMBA SDK](https://www.alliedvision.com/en/products/software.html) with Lua

## Prerequisites

```sh
wget "https://cdn.alliedvision.com/fileadmin/content/software/software/Vimba/Vimba_v2.1_Linux.tgz"
tar xvvf Vimba_v2.1_Linux.tgz
cd Vimba_2_1
cp VimbaC/Include/* /usr/local/include/
cp VimbaC/DynamicLib/x86_64bit/libVimbaC.so /usr/local/lib/
sudo ldconfig
```

## Installation

Install:
```sh
luarocks make
```
