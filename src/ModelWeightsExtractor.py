import tensorflow as tf

# wczytanie zapisanego modelu
model = tf.keras.models.load_model("../resources/model.keras")

# model.summary()

# zapisanie wag
model.save_weights("../resources/model.weights.h5")
