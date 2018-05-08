#!/usr/bin/env node
var shm = require('bindings')('shm.node')

var name = "/test";

shm.remove(name);

var sz = 5120;
var buf_test

buf_test1 = shm.open(name, sz);
console.log('This should be an ArrayBuffer:', buf_test1);

buf_test2 = shm.open(name);
console.log('This should be a read-only ArrayBuffer:', buf_test2);

console.log("Lengths:", buf_test2.byteLength, buf_test1.byteLength)

// Test the garbage collection
buf_test1 = null;
buf_test2 = null;
if (typeof global.gc === 'function'){
  global.gc();
}

console.log("OK, I'm done!");
