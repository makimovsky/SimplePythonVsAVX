import tensorflow as tf
# import h5py
#
# with h5py.File('../../resources/model.weights.h5', 'r') as f:
#     print(list(f["layers\\conv2d"]["vars"]["0"].shape))


# wczytanie zapisanego modelu
model = tf.keras.models.load_model("../../resources/model2.keras")

# wydobycie wag
conv_output_weights, conv_output_biases = model.layers[0].get_weights()
dense_output_weights, dense_output_biases = model.layers[3].get_weights()
output_logits_weights, output_logits_biases = model.layers[4].get_weights()


print("Przed przekształceniem:")
print(conv_output_biases.shape)
print(conv_output_weights.shape)
print(dense_output_biases.shape)
print(dense_output_weights.shape)
print(output_logits_biases.shape)
print(output_logits_weights.shape)

# struktura ParamsFile
cnn_params = {
    'convoutputBiases': conv_output_biases,
    'convoutputWeights': tf.transpose(conv_output_weights, perm=[3, 2, 0, 1]).numpy().flatten(),
    'denseoutputBiases': dense_output_biases,
    'denseoutputWeights': tf.transpose(dense_output_weights, perm=[1, 0]).numpy().flatten(),
    'outputlogitsBiases': output_logits_biases,
    'outputlogitsWeights': tf.transpose(output_logits_weights, perm=[1, 0]).numpy().flatten(),
}

print("\nPo przekształceniu:")
print(cnn_params.get("convoutputBiases").shape)
print(cnn_params.get("convoutputWeights").shape)
print(cnn_params.get("denseoutputBiases").shape)
print(cnn_params.get("denseoutputWeights").shape)
print(cnn_params.get("outputlogitsBiases").shape)
print(cnn_params.get("outputlogitsWeights").shape)

# zapisanie binarne wag do pliku ParamsFile
with open("../../resources/ParamsFile21", "wb") as file:
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
