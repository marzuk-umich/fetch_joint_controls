params = 1


def global_test():
    global params
    params = 100
    print(params)

global_test()
print(params)
