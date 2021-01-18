# 多雷达校准 #
## 用途 ##
用于多雷达场景下，某个雷达不准确的情况，发现转换矩阵（4*4），并批量实施转换

## 编译 ##
```
cmake .
make
```

## 运行 ##

根据pcd中4个雷达，找到雷达3相对雷达1、2、4的转换矩阵并输出

```
./registrator xxx.pcd
```

然后将转换矩阵替换到transform.cpp中，并重新编译

单帧转换

```
./transform 原pcd 新pcd 
```

注意新pcd已经转为二进制，并去掉了label

批量转换，修改run.py中的目录

```
python run.py
```