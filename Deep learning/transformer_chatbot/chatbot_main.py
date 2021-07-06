import os
# suppress warnings about dlls
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
from tensorflow.compat.v1 import layers
import numpy as np
import argparse
from transformer import Transformer
from preprocess_data import get_dataset, preprocess_sentence
from testing import evaluate

VOCAB_SIZE = 2**13
NUM_LAYERS = 2
NUM_UNITS = 512
D_MODEL = 256
NUM_HEADS = 8
UNITS = 512
DROPOUT = 0.1
NUM_EPOCHS = 20
BATCH_SIZE = 64

optimizer = tf.compat.v1.train.AdamOptimizer(learning_rate=0.001, beta1=0.9, beta2=0.98, epsilon=1e-9)

# https://leakyrelu.com/2020/01/01/difference-between-categorical-and-sparse-categorical-cross-entropy-loss-function/
# def SparseCategoricalCrossentropy(y_true : tf.Tensor, y_predict : tf.Tensor) -> (tf.Tensor):

#     num_samples = y_predict.shape[0]
#     return -(tf.reduce_sum(y_predict[tf.range(num_samples), y_true]))

def get_args() -> (argparse.Namespace):
    	
	parser = argparse.ArgumentParser()
	parser.add_argument( \
		'--max_samples',
		default=25000,
		type=int,
		help='maximum number of conversation pairs to use')
	parser.add_argument( \
		'--max_length', default=40, type=int, help='maximum sentence length')
	parser.add_argument('--batch_size', default=BATCH_SIZE, type=int)
	parser.add_argument('--num_layers', default=NUM_LAYERS, type=int)
	parser.add_argument('--num_units', default=NUM_UNITS, type=int)
	parser.add_argument('--d_model', default=D_MODEL, type=int)
	parser.add_argument('--num_heads', default=NUM_HEADS, type=int)
	parser.add_argument('--dropout', default=DROPOUT, type=float)
	parser.add_argument('--activation', default='relu', type=str)
	parser.add_argument('--epochs', default=NUM_EPOCHS, type=int)

	hparams = parser.parse_args()
	return hparams

def loss_function(y_true : tf.Tensor, y_predict : tf.Tensor) -> (tf.Tensor):

    mask = tf.math.logical_not(tf.math.equal(y_true, 0))
    
    # loss = SparseCategoricalCrossentropy(y_true, y_pred)
    loss = tf.keras.losses.SparseCategoricalCrossentropy()(y_true, y_predict)
    mask = tf.cast(mask, dtype=loss.dtype)
    loss *= mask

    return tf.reduce_sum(loss)/tf.reduce_sum(mask)

def accuracy(y_true : tf.Tensor, y_pred : tf.Tensor) -> (tf.Tensor):

    # ensure labels have shape (batch_size, MAX_LENGTH - 1)
    y_true = tf.reshape(y_true, shape=(-1, MAX_LENGTH - 1))
    accuracy = tf.metrics.SparseCategoricalAccuracy()(y_true, y_pred)
    return accuracy

def train_step(model : Transformer, x : tf.Tensor, dec_inputs : tf.Tensor, y_true : tf.Tensor) -> (tf.Tensor, tf.Tensor):
    
    with tf.GradientTape() as tape:
        y_predict = model(x, dec_inputs, training=True)
        loss = loss_function(y_true, y_predict)
        # l2_losses = model.get_l2_loss(loss.shape)
        # loss = tf.math.add(loss, 1e-5*l2_losses)

    # apply gradient
    dLoss = tape.gradient(loss, model.trainable_variables)
    optimizer.apply_gradients(zip(dLoss, model.trainable_variables))

    # loss
    training_loss = tf.reduce_mean(loss)

    return training_loss, y_predict

if __name__ == "__main__":

    hparams = get_args()

    dataset, tokenizer = get_dataset(hparams)

    model = Transformer(VOCAB_SIZE, NUM_LAYERS, UNITS, D_MODEL, NUM_HEADS, DROPOUT)
    training_losses, training_accuracy = {}, {}

    ### TRAIN ###

    for i in range(NUM_EPOCHS):

        y_true, y_predict = None, None

        for step, (x_batch_train, y_batch_train) in enumerate(dataset):

            y_true = y_batch_train
            training_loss, y_predict = train_step(model, x_batch_train['inputs'], \
                x_batch_train['dec_inputs'], y_batch_train)        
        
        training_losses[i] = training_loss
        training_accuracy[i] = accuracy(y_true, y_predict)

        print("Epoch", i+1, ", loss =", training_losses[i], ", accuracy =", training_accuracy[i])
    

    ### TEST ###

    evaluate(hparams, model, tokenizer)