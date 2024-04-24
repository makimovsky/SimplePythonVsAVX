import tensorflow as tf
import numpy as np

model = tf.keras.models.load_model('../../resources/model4.keras')

conv_output_weights, conv_output_biases = model.layers[0].get_weights()
dense_output_weights, dense_output_biases = model.layers[3].get_weights()
output_logits_weights, output_logits_biases = model.layers[4].get_weights()

conv_output_biases[:] = 1
conv_output_weights[:] = 1
dense_output_biases[:] = 1
dense_output_weights[:] = 1
#output_logits_biases[:] = 1
#output_logits_weights[:] = 1

model.layers[0].set_weights([conv_output_weights, conv_output_biases])
model.layers[3].set_weights([dense_output_weights, dense_output_biases])
model.layers[4].set_weights([output_logits_weights, output_logits_biases])

image_file = '../../resources/image0'
image = np.fromfile(image_file, dtype=np.float32)
image = image.reshape((1, 1, 28, 28)).astype(np.float32)

result = model.predict(image)

print(result)
