import os
# suppress warnings about dlls
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
from tensorflow.compat.v1 import layers
from datetime import datetime
import numpy as np
from transformer import Transformer
from chatbot_preprocess_data import get_cmdc_dataset, preprocess_sentence, get_args
from chatbot_testing import evaluate

VOCAB_SIZE = 2**13
NUM_LAYERS = 2
NUM_UNITS = 512
D_MODEL = 256
NUM_HEADS = 8
DROPOUT = 0.1
NUM_EPOCHS = 20
BATCH_SIZE = 64
MAX_LENGTH = 40

cwd = os.getcwd()
ckpt_dir = os.path.join(cwd, "transformer_chatbot\\saved_checkpoints\\")
ckpt_files = []

params = {
    'vocab_size' : VOCAB_SIZE,
    'num_layers' : NUM_LAYERS,
    'num_units' : NUM_UNITS,
    'd_model' : D_MODEL,
    'num_heads' : NUM_HEADS,
    'dropout' : DROPOUT,
    'num_epochs' : NUM_EPOCHS,
    'batch_size' : BATCH_SIZE,
    'max_length' : MAX_LENGTH
}

optimizer = tf.compat.v1.train.AdamOptimizer(learning_rate=0.001, beta1=0.9, beta2=0.98, epsilon=1e-9)

def save_model(checkpoint: tf.train.Checkpoint, ckpt_prefix : str) -> ():    
    ckpt_path = checkpoint.save(ckpt_prefix)
    ckpt_files.append(ckpt_path)

def load_model(checkpoint : tf.train.Checkpoint, ckpt_prefix : str) -> ():
    status = checkpoint.restore(ckpt_prefix)

def setup_checkpoints(name : str, model : tf.Module, clear_existing_files : bool=False) -> (tf.train.Checkpoint, str):

    if not os.path.exists(ckpt_dir):
        os.makedirs(ckpt_dir)

    if clear_existing_files:
        filelist = [ f for f in os.listdir(ckpt_dir) ]
        for f in filelist:
            os.remove(os.path.join(ckpt_dir,f))

    checkpoint = tf.train.Checkpoint(optimizer=optimizer, model=model)
    date = datetime.today().strftime('%Y-%m-%d')
    ckpt_prefix = os.path.join(ckpt_dir, "ckpt_" + name + "_" + date)

    return checkpoint, ckpt_prefix

def loss_function(y_true : tf.Tensor, y_predict : tf.Tensor) -> (tf.Tensor):

    mask = tf.math.logical_not(tf.math.equal(y_true, 0))
    
    loss = tf.keras.losses.SparseCategoricalCrossentropy()(y_true, y_predict)
    mask = tf.cast(mask, dtype=loss.dtype)
    loss *= mask

    return tf.reduce_sum(loss)/tf.reduce_sum(mask)

def accuracy(y_true : tf.Tensor, y_pred : tf.Tensor) -> (tf.Tensor):

    assert y_true.shape[1] == y_pred.shape[1]
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

def train_model() -> (str):

    hparams = get_args(params)

    dataset, tokenizer = get_cmdc_dataset(hparams)

    model = Transformer(VOCAB_SIZE, NUM_LAYERS, NUM_UNITS, D_MODEL, NUM_HEADS, DROPOUT)
    training_losses, training_accuracy = [], []

    checkpoint, ckpt_prefix = setup_checkpoints("chatbot", model, clear_existing_files=False)

    ### TRAIN ###

    for i in range(NUM_EPOCHS):

        y_true, y_predict = None, None

        for step, (x_batch_train, y_batch_train) in enumerate(dataset):

            y_true = y_batch_train
            training_loss, y_predict = train_step(model, x_batch_train['inputs'], \
                x_batch_train['dec_inputs'], y_batch_train)        
        
        training_losses.append(training_loss)
        acc = accuracy(y_true, y_predict)
        training_accuracy.append(acc)

        # save checkpoint
        save_model(checkpoint, ckpt_prefix)

        print("Epoch", i+1, ", loss =", training_loss.numpy(), ", accuracy =", acc.numpy())    

    ### TEST ###

    # find and load model with best params
    iBest = training_losses.index(min(training_losses))  
    if len(ckpt_files) > 0:
        if iBest > len(ckpt_files) - 1:
            iBest = len(ckpt_files) - 1
        best_ckpt = ckpt_files[iBest]
        print("Best checkpoint file is", best_ckpt)
        load_model(checkpoint, best_ckpt)

    # test
    evaluate(hparams, model, tokenizer)

    return best_ckpt


def run_trained_model(checkpoint_name : str) -> ():

    hparams = get_args(params)

    dataset, tokenizer = get_cmdc_dataset(hparams)

    model = Transformer(VOCAB_SIZE, NUM_LAYERS, NUM_UNITS, D_MODEL, NUM_HEADS, DROPOUT)
    training_losses, training_accuracy = [], []

    checkpoint, ckpt_prefix = setup_checkpoints("chatbot", model, clear_existing_files=False)

    load_model(checkpoint, checkpoint_name)
    evaluate(hparams, model, tokenizer)

if __name__ == "__main__":

    train_model()

    # run_trained_model("C:\\Users\\ashli\\Documents\\GIT - personal_projects\\Deep learning\\transformer_chatbot\\saved_checkpoints\\ckpt_chatbot_2021-07-26-6")