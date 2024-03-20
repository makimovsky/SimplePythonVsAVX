import numpy as np
import tensorflow as tf

# wczytanie zapisanego modelu
model = tf.keras.models.load_model("../../resources/model.keras")

# wydobycie wag
conv_output_weights, conv_output_biases = model.layers[0].get_weights()
dense_output_weights, dense_output_biases = model.layers[3].get_weights()
output_logits_weights, output_logits_biases = model.layers[4].get_weights()

# struktura ParamsFile
cnn_params = {
    'convoutputBiases': conv_output_biases,
    'convoutputWeights': np.transpose(conv_output_weights, (3, 2, 0, 1)),
    'denseoutputBiases': dense_output_biases,
    'denseoutputWeights': np.transpose(dense_output_weights, (1, 0)),
    'outputlogitsBiases': output_logits_biases,
    'outputlogitsWeights': np.transpose(output_logits_weights, (1, 0))
}

print(cnn_params.get("convoutputBiases").shape)
print(cnn_params.get("convoutputWeights").shape)
print(cnn_params.get("denseoutputBiases").shape)
print(cnn_params.get("denseoutputWeights").shape)
print(cnn_params.get("outputlogitsBiases").shape)
print(cnn_params.get("outputlogitsWeights").shape)


# zapisanie binarne wag do pliku ParamsFile
with open("../../resources/ParamsFile", "wb") as file:
    for key, value in cnn_params.items():
        value.tofile(file)
