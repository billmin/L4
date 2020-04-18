'''
六种常见的隶属度函数
矩阵
梯形
抛物线
高斯
柯西
伽马
'''
from math import exp
import numpy as np
import matplotlib.pyplot as plt


class MembershipDegree(object):
    def no_mold(self,mold):
        # 选择模式是否存在
        print('no this mold: {0} ???\n we have 0/1/2 represent small/medium/large'.format(mold))
        exit(1)

    def y_array(self,f,x):
        y=[]
        for xx in x:
           y.append(f(xx))
        y=np.array(y)
        return y
    def huahua(self,x,y):
        plt.plot(x, y)
        plt.ylim(-0.2, 1.2)
        plt.xlim(x[0] - (max(x) - min(x)) * 0.1, x[-1] + (max(x) - min(x)) * 0.1)
        # plt.axes([x[0]-(max(x)-min(x))*0.1,x[-1]+(max(x)-min(x))*0.1,-0.2,1.2])
        plt.grid()
        
        
class Matrix(MembershipDegree):
    def __init__(self,a,b=None,mold=0):
        super().__init__()
        if (mold == 1):
            assert b != None
        self.a=a
        self.b=b
        self.mold=mold
    def matrix(self,x):
        a,b,mold=self.a,self.b,self.mold
        # 矩阵型
        if(mold==1):
            # 中间型
            if (a<=x<=b):
                u = 1
            else:
                u = 0
        elif(mold==0):
            # 偏小型
            if (x<=a):
                u=1
            else:
                u=0
        elif(mold==2):
            # 偏大型
            if (x<a):
                u=0
            else:
                u=1
        else:
            super().no_mold(mold)
            return
        return u

class Trapezoid(MembershipDegree):
    def __init__(self,a,b,c=None,d=None,mold=0):
        super().__init__()
        if (mold == 1):
            assert c != None and d != None
        self.a,self.b,self.c,self.d,self.mold=a,b,c,d,mold
    def trapezoid(self,x):
        a, b, c, d, mold=self.a,self.b,self.c,self.d,self.mold
        # 梯形
        if(mold==1):
            assert c!=None and d!=None
            # 中间型
            if(a<=x<=b):
                u=(x-a)/(b-a)
            elif(b<=x<=c):
                u=1
            elif(c<=x<=d):
                u=(d-x)/(d-c)
            else:
                u=0
        elif(mold==0):
            # 小
            if(x<=a):
                u=1
            elif(x>b):
                u=0
            else:
                u=(b-x)/(b-a)
        elif(mold==2):
            # 大
            if (x <= a):
                u = 0
            elif (x > b):
                u = 1
            else:
                u = 1-(b - x) / (b - a)
        else:
            super().no_mold(mold)
            return
        return u

class Paracurve(MembershipDegree):
    def __init__(self,a,b,c=None,d=None,mold=0,k=3):
        super().__init__()
        self.a, self.b, self.c, self.d, self.mold,self.k= a, b, c, d, mold,k
        self.temp = Trapezoid(a, b, c, d, mold)
    def paracurve(self,x):
        # k次抛物线 哈哈-->>就是在梯形型基础上对每一个结果k次方
        a, b, c, d, mold, k=self.a, self.b, self.c, self.d, self.mold, self.k
        u=self.temp.trapezoid(x)**k
        return u

class Gauss(MembershipDegree):
    def __init__(self,a=0,mold=0,theta=1):
        self.a,self.mold,self.theta=a,mold,theta
    def gauss(self,x):
        a, mold, theta=self.a,self.mold,self.theta
        # theta:方差
        # a:均值

        # 高斯型
        f=lambda :exp(-((x-a)/theta)**2)
        if(mold==1):
            u=f()
        elif(mold==0):
            if(x<=a):
                u=1
            else:
                u=f()
        elif(mold==2):
            if(x<=a):
                u=0
            else:
                u=1-f()
        else:
            super().no_mold(mold)
            return
        return u

class Cauchy(MembershipDegree):
    def __init__(self,a,mold=0,alpha=1,beta=2):
        assert alpha > 0 and beta > 0
        if (mold == 1):
            assert beta % 2 == 0
        self.a,self.mold,self.alpha,self.beta=a,mold,alpha,beta
    def cauchy(self,x):
        '''
        柯西型
        :param x:
        :param a: 绝对值开始/结束
        :param mold:
        :param alpha: 可以选1/sqrt(a)
        :param beta: 变化速度 越大越快 1为线性
        :return:
        '''
        a, mold, alpha, beta=self.a,self.mold,self.alpha,self.beta
        f=lambda b:1/(1+alpha*(x-alpha)**b)
        if(mold==1):
            u=f(beta)
        elif(mold==0):
            if(x<=a):
                u=1
            else:
                u=f(beta)
        elif(mold==2):
            if(x<=a):
                u=0
            else:
                u=f(-beta)
        else:
            super().no_mold(mold)
            return
        return u

class Gamma(MembershipDegree):
    def __init__(self,a,b=None,mold=0,k=2):
        if (mold == 1):
            assert b != None
        self.a,self.b,self.mold,self.k=a,b,mold,k
    def gamma(self,x):
        '''
        伽马型
        :param x:
        :param a:
        :param b:
        :param mold:
        :param k:
        :return:
        '''
        a, b, mold, k= self.a, self.b, self.mold, self.k
        f=lambda :exp(-k*(x-a))
        if(mold==1):
            if(x<a):
                u=exp(k*(x-a))
            elif(x>b):
                u=exp(-k*(x-b))
            else:
                u=1
        elif(mold==0):
            if(x<=a):
                u=1
            else:
                u=f()
        elif(mold==2):
            if(x<=a):
                u=0
            else:
                u=1-f()
        else:
            super().no_mold(mold)
            return
        return u


if __name__ == '__main__':

    # 画图测试
    x=np.arange(0,10,0.01)
    # 实例化一个伽马
    g=Gauss(a=3,mold=1)
    # 用父类函数运算哈哈哈
    y=g.y_array(g.gauss,x)
    g.huahua(x,y)
    # plt.savefig('test')
    plt.show()
    pass
