import tensorflow as tf
import numpy as np
import time

model = tf.keras.models.load_model('model.keras')

start = time.time()
for i in range(1000):
    image_file = f'../../resources/images/image{i}'
    image = np.fromfile(image_file, dtype=np.float32)
    image = image.reshape((1, 28, 28, 1)).astype(np.float32)

    model.predict(image)

t = time.time() - start

with open("../../results/pythontime.txt", "w") as f:
    f.write(str(t))
f.close()
