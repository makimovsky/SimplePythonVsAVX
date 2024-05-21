import tensorflow as tf
import numpy as np
import time

model = tf.keras.models.load_model('../resources/model3.keras')

start = time.time()
with open("../resources/pythontime.txt", "w") as f:
    for i in range(1000):
        image_file = f'../resources/images/image{i}'
        image = np.fromfile(image_file, dtype=np.float32)
        image = image.reshape((1, 28, 28, 1)).astype(np.float32)

        model.predict(image)

        if i % 9 == 0:
            t = time.time() - start
            f.write(str(t) + '\n')

f.close()
