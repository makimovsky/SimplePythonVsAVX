import tensorflow as tf
from tensorflow.keras import layers, datasets, models

# pobranie zbioru danych do uczenia i testowania
(train_images, train_labels), (test_images, test_labels) = datasets.fashion_mnist.load_data()

# normalizacja wartosci pikseli
train_images, test_images = train_images / 255.0, test_images / 255.0

# przekształcenie danych wejściowych na format NCHW
train_images = train_images.reshape(train_images.shape[0], 1, 28, 28)
test_images = test_images.reshape(test_images.shape[0], 1, 28, 28)

# utworzenie modelu
model = models.Sequential()
model.add(layers.Conv2D(32, (3, 3), activation='relu', input_shape=(1, 28, 28), data_format='channels_first'))
model.add(layers.MaxPooling2D((2, 2), data_format='channels_first'))

model.add(layers.Flatten())
model.add(layers.Dense(64, activation='relu'))
model.add(layers.Dense(10))

# kompilacja i trenowanie modelu
model.compile(optimizer='adam', loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
              metrics=['accuracy'])

model.fit(train_images, train_labels, epochs=10, validation_data=(test_images, test_labels))

# dokładność modelu
score = model.evaluate(test_images,  test_labels, return_dict=True)

print("\nModel accuracy for test data:", score["accuracy"])

# zapisanie modelu
model.save('../../resources/model4.keras')
