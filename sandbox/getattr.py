class A(object):
    def run(self):
        return "run"


a = A()

print(hasattr(a, "run"))         # True
print(getattr(a, "run"))         # <bound method A.run of <__main__.A object at 0x0000000002A57160>>
print(getattr(a, "run")())       # run
