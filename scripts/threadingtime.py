import threading


def print_cube(num):
    print("Cube: {}" .format(num * num * num))
    for i in range(10): 
        print("thread2")

def print_square(num):
    print("Google: {}" .format(num * num * num))

    for i in range(10): 
        print("thread1")


if __name__ =="__main__":
    t1 = threading.Thread(target=print_square, args=(10,))
    t2 = threading.Thread(target=print_cube, args=(10,))

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    print("Done!")
