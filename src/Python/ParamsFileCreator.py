import numpy as np
import tensorflow as tf

# wczytanie zapisanego modelu
model = tf.keras.models.load_model("../../resources/model3.keras")

# wydobycie wag
conv_output_weights, conv_output_biases = model.layers[0].get_weights()
dense_output_weights, dense_output_biases = model.layers[3].get_weights()
output_logits_weights, output_logits_biases = model.layers[4].get_weights()

#conv_output_biases[:] = 1
#conv_output_weights[:] = 1
#dense_output_biases[:] = 1
#dense_output_weights[:] = 1
#output_logits_biases[:] = 1
#output_logits_weights[:] = 1

r = tf.random.Generator.from_seed(111)
dense_output_weights = r.normal(shape=(5408, 64)).numpy()

model.layers[0].set_weights([conv_output_weights, conv_output_biases])
model.layers[3].set_weights([dense_output_weights, dense_output_biases])
model.layers[4].set_weights([output_logits_weights, output_logits_biases])

image_file = '../../resources/image0'
image = np.fromfile(image_file, dtype=np.float32)
image = image.reshape((1, 28, 28, 1)).astype(np.float32)

result = model.predict(image)

print(result)

dense_output_weights = dense_output_weights.reshape(13, 13, 32, 64)
conv_output_weights = conv_output_weights / -2.685

#struktura ParamsFile
cnn_params = {
        'convoutputBiases': conv_output_biases.flatten(), # dobrze
        'convoutputWeights': tf.transpose(conv_output_weights).numpy().flatten(), # dobrze?
        'denseoutputBiases': dense_output_biases.flatten(), # dobrze
        'denseoutputWeights': tf.transpose(dense_output_weights).numpy().flatten(),
        'outputlogitsBiases': output_logits_biases.flatten(), # dobrze
        'outputlogitsWeights': tf.transpose(output_logits_weights).numpy().flatten(), # dobrze
}

# zapisanie binarne wag do pliku ParamsFile
with open("../../resources/ParamsFile4", "wb") as file:
    for key, value in cnn_params.items():
        value.tofile(file)
