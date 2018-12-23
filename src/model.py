from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Dense, Flatten
from keras.callbacks import ModelCheckpoint, TensorBoard
import os
from keras.preprocessing.image import ImageDataGenerator


def get_model():
    model = Sequential()
    model.add(Conv2D(8, (3, 3), activation='relu', padding='same',
                     input_shape=(196, 196, 3)))
    model.add(MaxPooling2D((2, 2)))
    model.add(Conv2D(16, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D(2, 2))
    model.add(Conv2D(32, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D(2, 2))
    model.add(Flatten())
    model.add(Dense(32, activation="relu"))
    # final layer will output probabilities for 7 gestures
    model.add(Dense(7, activation='softmax'))

    model.compile(optimizer='rmsprop',
                  loss='categorical_crossentropy',
                  metrics=['accuracy'])

    return model

"""
the dataset folder must be in the following format:
    /dataset
        /train
            /gesture1
            /gesture2
            ...
        / test
            /gesture1
            /gesture2
"""

dataset_dir = "/path/to/dataset"
train_dir = os.path.join(dataset_dir, 'train')
val_dir = os.path.join(dataset_dir, 'valid')


def get_training_data_generator():
    train_datagen = ImageDataGenerator(rescale=1./255)
    train_generator = train_datagen.flow_from_directory(
        directory=train_dir,  # directory to read training images from
        target_size=(196, 196),  # all images resized to (196,196)
        batch_size=64,  # each batch consists of 64 samples
        class_mode='categorical',
        interpolation='bicubic'
    )
    return train_generator

def get_validation_data_generator():
    valid_datagen = ImageDataGenerator(rescale=1/255.0)
    validation_generator = valid_datagen.flow_from_directory(
        directory=val_dir,  # directory to read validation images from
        target_size=(196, 196),  # all images resized to (196, 196)
        batch_size=64,  # each batch consists of 64 samples
        class_mode='categorical',
        interpolation='bicubic'
    )
    return validation_generator


if __name__ == "__main__":

    recognizer = get_model()
    train_generator = get_training_data_generator()
    validation_generator = get_validation_data_generator()

    step_size_train = train_generator.n // train_generator.batch_size
    step_size_valid = validation_generator.n // validation_generator.batch_size
    tensorboard = TensorBoard(log_dir="./logs", histogram_freq=0,
                              batch_size=64, write_graph=True,
                              write_images=False)
    model_check = ModelCheckpoint("/path/to/weights.{epoch:02d}-"
                                  "{val_acc:.2f}.hdf5", monitor='val_acc',
                                  verbose=1, save_best_only=True, mode='max')

    history = recognizer.fit_generator(
        train_generator,
        steps_per_epoch=step_size_train,
        epochs=100,
        validation_data=validation_generator,
        validation_steps=step_size_valid,
        callbacks=[tensorboard, model_check]
    )
