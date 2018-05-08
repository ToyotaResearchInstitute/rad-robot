ffi=require'ffi';C=ffi.C
n=4
sz=ffi.sizeof'int'*n
x=ffi.new("int[?]", n)
x[0]=3
p=tonumber(ffi.cast('intptr_t', ffi.cast('void *', x)))
py=require'python'
torch=py.import'torch'
b=py.buffer(p, sz)
s=torch.IntStorage.from_buffer(b, 'native')
=s
t=torch.IntTensor(s)