# 加载参数进参数服务器
```
<rosparam file="$(find engineer_middleware)/config/steps_list.yaml" command="load
ns="engineer_middleware"/>
```
先要在.launch文件里load参数文件
# 在程序中调用
通常加载后直接使用相同名字的值就可以
```
getParam(nh, " ", 1);
```
这一条命令就可以直接作为一个具体的值