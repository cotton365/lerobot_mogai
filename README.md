### 将jaka和mycobot接入lerobot项目
1.关键文件在src/lerobot/robots/里面可以找到jaka的从臂代码和mycobot的主臂从臂代码
2.在so101_test下是写好的遥操脚本，校准等前置任务脚本也有
3.注意jaka使用TCP/IP协议，这也是为什么这里使用了脚本而不是接入已有框架
4.jaka原本没有夹爪，夹爪部分，也就是这里的第七个关节没有做映射处理，需要按照自己需求编写
