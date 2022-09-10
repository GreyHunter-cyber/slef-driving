 proto编译说明
 ===============


 proto 文件可以编译各种语言版本，包括 C 和 C++。

 1. 编译 c 版本。`sudo apt-get install libprotobuf-c-dev protobuf-c-compiler` 安装依赖工具，`protoc-c robot.proto --c_out=.` 编译proto文件到当前目录，生成 `robot.pb-c.h` 和 `robot.pb-c.c` 文件。
 
 2. 编译 c++ 版本。`sudo apt-get install libprotobuf-dev protobuf-compiler` 安装依赖工具，`protoc robot.proto --cpp_out=.` 编译proto文件到当前目录，生成 `robot.pb.h` 和 `robot.pb.cc` 文件。

