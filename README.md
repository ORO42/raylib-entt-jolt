```
mkdir build
cd build
cmake ..
make
./emergent-command
```

- Must clone vendor repos into vendor

`mkdir build && cd build && cmake .. && make && ./emergent-command`

`make && ./emergent-command`

- Make sure cpp version is set in `c_cpp_properties.json`

# TODO

[ ] Make capsule heights consistent
[ ] Fix character capsule drawing (should be same as capsule physics body drawing)
[ ] Fix character controller jump
[ ] Fix character controller move directions
[ ] Fix when physics capsule is pushed, looks like its falling off an invisible object
[ ] Draw position debug markers
