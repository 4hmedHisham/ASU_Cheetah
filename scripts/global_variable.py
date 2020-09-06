import NodeMaker as create
import rospy

glbl_var=create.ros('p','init','init')
shoes=25
l1 =245
l2 =208.4+shoes
a = 112.75
initalheight=390
stride=60
def modify_vars():
    global l1
    global l2
    global a
    global initalheight
    global stride
    flag=raw_input("Please enter Length of hip segment or d for default ")
    if flag!="d":
        l1=flag
        print("YO")
    print('l1 = '+str(l1))

    flag2=raw_input("Please enter length of shank segment+shoes if any or d for default ")
    if flag2!="d":
        l2=flag2
    print('l2 = '+str(l2))
    flag3=raw_input("Please input a raw value or d for default ")
    if flag3!="d":
        a=flag3
    print('a = '+str(a))
    flag4=raw_input("Please input stride length or d for default ")
    if flag4!="d":
        stride=flag4
    print('stride = '+str(stride))
modify_vars()
while not rospy.is_shutdown():
    glbl_var.ros_publish([l1,l2,a,initalheight,stride])
