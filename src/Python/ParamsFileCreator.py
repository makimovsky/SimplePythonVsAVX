import numpy as np
import tensorflow as tf

# wczytanie zapisanego modelu
model = tf.keras.models.load_model("../../resources/model4.keras")

# wydobycie wag
conv_output_weights, conv_output_biases = model.layers[0].get_weights()
dense_output_weights, dense_output_biases = model.layers[3].get_weights()
output_logits_weights, output_logits_biases = model.layers[4].get_weights()

#dense_output_weights = np.reshape(dense_output_weights, (13, 13, 32, 64))
"""
params = [conv_output_biases, conv_output_weights, dense_output_biases, dense_output_weights, output_logits_biases, output_logits_weights]

conv_output_biases[:] = 1
conv_output_weights[:] = 1
dense_output_biases[:] = 1
dense_output_weights[:] = 1
output_logits_biases[:] = 1
output_logits_weights[:] = 1

output_logits_weights[1][7] = 2

output_logits_weights = output_logits_weights[::-1, ::-1]

np.random.seed(2256)

for i in params:
    elems = i.size

    num_twos = int(elems * 0.3)

    rand_elems = np.random.choice(elems, num_twos, replace=False)

    arr = np.unravel_index(rand_elems, i.shape)

    i[arr] = 2
"""

print("Przed przekształceniem:")
print(conv_output_biases.shape)
print(conv_output_weights.shape)
print(dense_output_biases.shape)
print(dense_output_weights.shape)
print(output_logits_biases.shape)
print(output_logits_weights.shape)

conv_output_weights = conv_output_weights / -2.34

#struktura ParamsFile
cnn_params = {
        'convoutputBiases': conv_output_biases.flatten(), # dobrze
        'convoutputWeights': tf.transpose(conv_output_weights).numpy().flatten(), # dobrze?
        'denseoutputBiases': dense_output_biases.flatten(), # dobrze
        'denseoutputWeights': dense_output_weights.flatten(), #tf.transpose(dense_output_weights).numpy().flatten(),
        'outputlogitsBiases': output_logits_biases.flatten(), # dobrze
        'outputlogitsWeights': tf.transpose(output_logits_weights).numpy().flatten(), # dobrze
}


print("\nPo przekształceniu:")
print(cnn_params.get("convoutputBiases").shape)
print(cnn_params.get("convoutputWeights").shape)
print(cnn_params.get("denseoutputBiases").shape)
print(cnn_params.get("denseoutputWeights").shape)
print(cnn_params.get("outputlogitsBiases").shape)
print(cnn_params.get("outputlogitsWeights").shape)

# zapisanie binarne wag do pliku ParamsFile
with open("../../resources/ParamsFile4", "wb") as file:
    for key, value in cnn_params.items():
        value.tofile(file)

# Przed przekształceniem:
# (32,)
# (3, 3, 1, 32)
# (64,)
# (5408, 64)
# (10,)
# (64, 10)
#
# Po przekształceniu:
# (32,)
# (288,)
# (64,)
# (346112,)
# (10,)
# (640,)
