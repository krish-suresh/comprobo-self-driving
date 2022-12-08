import importlib
encoder = importlib.import_module('rpi-rotary-encoder-python.encoder')

def valueChanged(value):
    print(value)

e1 = encoder.Encoder(26, 19, callback=valueChanged)