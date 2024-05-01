import tensorflow as tf
import numpy as np

model = tf.keras.models.load_model('../../resources/model4.keras')

conv_output_weights, conv_output_biases = model.layers[0].get_weights()
dense_output_weights, dense_output_biases = model.layers[3].get_weights()
output_logits_weights, output_logits_biases = model.layers[4].get_weights()
"""
params = [conv_output_biases, conv_output_weights, dense_output_biases, dense_output_weights, output_logits_biases, output_logits_weights]

conv_output_biases[:] = 1
conv_output_weights[:] = 1
dense_output_biases[:] = 1
dense_output_weights[:] = 1
output_logits_biases[:] = 1
output_logits_weights[:] = 1

output_logits_weights[1][7] = 2


np.random.seed(2256)

for i in params:
    elems = i.size

    num_twos = int(elems * 0.3)

    rand_elems = np.random.choice(elems, num_twos, replace=False)

    arr = np.unravel_index(rand_elems, i.shape)

    i[arr] = 2

model.layers[0].set_weights([conv_output_weights, conv_output_biases])
model.layers[3].set_weights([dense_output_weights, dense_output_biases])
model.layers[4].set_weights([output_logits_weights, output_logits_biases])
"""
image_file = '../../resources/image0'
image = np.fromfile(image_file, dtype=np.float32)
image = image.reshape((1, 1, 28, 28)).astype(np.float32)

result = model.predict(image)

print(result)
