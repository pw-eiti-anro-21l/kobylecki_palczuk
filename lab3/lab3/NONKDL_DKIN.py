import yaml

def read_from_yaml(filename):
    with open(filename, 'r') as file:
        reader = yaml.load(file, Loader=yaml.FullLoader)
    return reader

def transformacja():
    yml = read_from_yaml('kobylecki_palczuk/lab2/urdf/rpy.yaml')

    for element in yml:
        a = yml[element]['a']
        alpha = yml[element]['alpha']
        theta = yml[element]['theta']
        x = x + a

def main():
    print('Hi from lab3.')


if __name__ == '__main__':
    main()
