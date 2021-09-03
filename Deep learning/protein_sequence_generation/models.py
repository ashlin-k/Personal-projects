import numpy as np
import random
from sklearn.linear_model import Lasso
from sklearn.model_selection import GridSearchCV
from sklearn.metrics import make_scorer
from scipy.stats import pearsonr
from datetime import datetime
import csv

import os
# suppress warnings about dlls
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
import tensorflow_addons as tfa
from tensorflow.keras import layers
from sklearn.model_selection import train_test_split

from math import log10
from aa_encoding import *

amino_acids = [
    'A', 'R', 'N', 'D', 'C', 'E', 'Q', 'G', 'H', 'I', 'L', 'K',
    'M', 'F', 'P', 'S', 'T', 'W', 'Y', 'V'
]

def read_sequences(filename : str) -> (dict):
    
    '''
    @param filename:        The relative path to the csv file containing the training data.
                            Each line should be in the format (id, sequence, fitness).
    @return dataset:        A dictionary of the dataset in the format { sequence(str) : fitness(float) }
    '''

    if not os.path.isfile(filename):
        print("Data file does not exist")
        return []
    
    seqs = {}
    with open(filename, newline='') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in csvreader:
            if len(row) < 3:
                print("Data file does not have the correct format")
                return {}
            try:
                # motif = Motif(row[0], row[1], float(row[2]))
                seqs[row[1]] = float(row[2]) # motif
            except:
                pass            

    return seqs

def pearson_r(y_true : np.array , y_predict : np.array) -> (float):

    '''
    @param y_true:          An array of the true outputs.
    @param y_predict:       An array of the predicted outputs. Should match the dimensions of y_true
    @return r:              The Pearson R correlation coefficient.
    '''
    r, pval = pearsonr(y_true, y_predict)
    return r

def hamming_distance(x1 : str, x2 : str) -> (float):
    
    '''
    @param x1:          A string sequence of length N.
    @param x2:          A string sequence of length N.
    @return HD:         The Hamming distance between x1 and x2
    '''

    if len(x1) != len(x2):
        return -1

    return sum(c1 != c2 for c1, c2 in zip(x1, x2))

class WuFitnessModel:

    def __init__(self, wt_seq : str="VDGV"):

        '''
        @param wt_seq:          A string sequence of the wild-type variant.
        '''

        # we are setting intercept = log(1) = 0, as described by Wu et. al.
        # we'll also temporarily set alpha = 1e-4, as described by Wu et. al.
        self.alpha = 1e-4
        self.fit_intercept = False
        self.model = Lasso(alpha=self.alpha, fit_intercept=self.fit_intercept)  

        self.wt_seq = ""
        if len(wt_seq) == 0:
            self.wt_seq = "VDGV"
        else:
            self.wt_seq = wt_seq
    
    def generate_x_entry(self, seq : str) -> (np.array):

        '''
        @param seq:         A string sequence of a variant. Should be same length as wt_seq.
        @return Xi:         The M, P, T encoding of the sequence compared to wt_seq, 
                            according to Wu et. al.
        '''

        Nm = 76
        Np = 2166
        Nt = 6859
        num_coeffs = Nm + Np + Nt
        Xi = np.zeros((num_coeffs,))
        
        if len(seq) != len(self.wt_seq):
            return Xi

        hd = hamming_distance(seq, self.wt_seq)
        m, p, t = 0, 0, 0
        if hd == 1:
            m = 1
        elif hd == 2:
            p = 2
        elif hd == 3:
            t = 3
        Xi[0:Nm] = m 
        Xi[Nm:Np] = p 
        Xi[Np:Nt] = t

        return Xi

    def construct_training_data(self, dataset : dict) -> (np.array, np.array):

        '''
        @param dataset:         A dictionary of format { sequence(str) : fitness(float) }
        @return X:              A 2D matrix containing the M, P and T variables for each variant.
        @return Y:              A 1D vector containing log10(fitness) for each variant.
        '''

        seqs = list(dataset.keys())
        fitness_scores = list(dataset.values())
        N = len(seqs)
        X = []
        Y = []

        for i in range(N):

            # exclude all lethal variants (fitness = 0)
            if fitness_scores[i] == 0.0 or len(seqs[i]) != len(self.wt_seq):
                continue

            Xi = self.generate_x_entry(seqs[i])
            X.append(Xi)

            Y.append(log10(fitness_scores[i]))
        
        X = np.array(X)
        Y = np.array(Y)
        
        return X, Y
    
    def train(self, dataset : dict, optimize_alpha : bool=True) -> (float):

        '''
        @param dataset:     A dictionary of format { sequence(str) : fitness(float) }
        @return R:          The Pearson R correlation coefficient.
        '''

        if len(dataset) == 0:
            return 0

        # convert dataset into X and Y matrices for Lasso linear regression
        Xtrain, Ytrain = self.construct_training_data(dataset)

        if optimize_alpha:

            # first use cross validation to find the optimal value of alpha, the L1 coefficient
            # https://towardsdatascience.com/linear-regression-models-4a3d14b8d368
            alpha = [1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 0.001, 0.01, 0.1, 1]
            param_grid = dict(alpha=alpha)
            # pearsonr_scorer = make_scorer(pearson_r)
            grid = GridSearchCV(estimator=self.model, param_grid=param_grid, scoring="neg_mean_squared_error", verbose=1, \
                n_jobs=-1, cv=10)
            grid_result = grid.fit(Xtrain, Ytrain)
            self.alpha = grid_result.best_params_['alpha']
        
        else:

            # predtermined best params from a prior fitting of model
            self.alpha = 1e-8
            self.fit_intercept = True

        # create model with optimized alpha param
        model_params = { 'alpha' : self.alpha, 'fit_intercept' : self.fit_intercept }
        self.model.set_params(**model_params)
        self.model.fit(Xtrain, Ytrain)
        Ypredict = self.model.predict(Xtrain)
        r = pearson_r(Ytrain, Ypredict)
        return r
    
    def predict(self, Xtest : str) -> (float):

        '''
        @param Xtest:       A string sequence of a variant.
        @return fitness:   The predicted fitness scores of each sequence.
        '''

        if len(Xtest) != len(self.wt_seq):
            return 0.0

        Xtest_encoded = self.generate_x_entry(Xtest)
        if Xtest_encoded.ndim != 2:
            Xtest_encoded = np.reshape(Xtest_encoded, (1,-1))
        log10_fitness = self.model.predict(Xtest_encoded)[0]
        fitness = 10**log10_fitness
        
        return fitness

class LinderFitnessModel(tf.Module):
    
    def __init__(self, name : str="LinderFitnessModel"):

        '''
        @param name:            Name of the model.
        '''
        
        super(LinderFitnessModel, self).__init__(name=name)

        # due to our smaller training set, we will reduce the number of parameters such that
        # num_params = dataset_size/10

        # we also set the seed for the GlorotUniform initializer to a constant so that
        # the results are repeatable
        
        self.conv1d_layer1 = layers.Conv1D(filters=7, kernel_size=2, activation='relu', \
            data_format='channels_last', kernel_initializer=tf.keras.initializers.GlorotUniform(seed=1))
        self.conv1d_layer2 = layers.Conv1D(filters=10, kernel_size=2, activation='relu', \
            data_format='channels_last', kernel_initializer=tf.keras.initializers.GlorotUniform(seed=1))
        self.dense_layer1 = layers.Dense(units=19, activation="relu", \
            kernel_initializer=tf.keras.initializers.GlorotUniform(seed=1))
        self.dropout = layers.Dropout(0.2)
        # self.maxpool_layer = layers.MaxPool1D(pool_size=2)
        self.gap_layer = y = tf.keras.layers.GlobalAveragePooling1D()
        self.dense_layer2 = layers.Dense(units=1, activation="sigmoid", \
            kernel_initializer=tf.keras.initializers.GlorotUniform(seed=1))

    def __call__(self, inputs : tf.Tensor, is_training : bool=True) -> (tf.Tensor):

        '''
        @param inputs:          One-hot encodings of the sequences. Expected dimensions
                                are (NUM_SEQ, SEQ_LENGTH, NUM_CHARS)
        @param is_training:     A flag indicate if the model is being called on training samples. 
                                If so, dropout will be applied. If not, then dropout will not be applied.
        '''

        x = self.conv1d_layer1(inputs)        
        x = self.conv1d_layer2(x)
        x = self.dense_layer1(x)

        if is_training:
            x = self.dropout(x)
        
        x = self.gap_layer(x)
        # x = self.maxpool_layer(x)
        x = self.dense_layer2(x)

        return tf.reshape(x, (x.shape[0],))

class LinderTrainer:

    def __init__(self):

        self.model = None # LinderFitnessModel()
        self.name = "LinderFitnessModel"

        # init optimizer, metrics and loss function
        self.loss_fn = tf.keras.losses.MeanSquaredError() # tf.keras.losses.KLDivergence()
        self.metric = tfa.metrics.r_square.RSquare()
        self.optimizer = tf.keras.optimizers.Adam(learning_rate=0.001)

        # init training params
        self.batch_size = 32
        self.num_epochs = 100

        # init validation variables
        self.val_losses, self.val_metrics = [], []

        # set up checkpointing
        cwd = os.getcwd()
        self.ckpt_dir = os.path.join(cwd, "saved_checkpoints\\")
        self.cv_best_ckpts = []
        date = datetime.today().strftime('%Y-%m-%d')
        self.ckpt_prefix = os.path.join(self.ckpt_dir, "ckpt_" + self.name + "_" + date)
        self.checkpoint = None
        self.best_ckpt = ""

        if not os.path.exists(self.ckpt_dir):
            os.makedirs(self.ckpt_dir)

        filelist = [ f for f in os.listdir(self.ckpt_dir) ]
        for f in filelist:
            os.remove(os.path.join(self.ckpt_dir,f))        

        self.is_trained = False
    
    def save_model(self, suffix : str="") -> (str):

        '''
        @param suffix:          A string to append to the end of the checkpoint prefix
        '''

        if self.checkpoint is not None:
            ckpt_path = self.checkpoint.save(self.ckpt_prefix + suffix)
            return ckpt_path
        else:
            print("Model is set to None. Could not save checkpoint.")  

    def load_model(self, ckpt_prefix : str) -> ():

        '''
        @param ckpt_prefix:         The checkpoint prefix
        '''

        if self.checkpoint is not None:
            status = self.checkpoint.restore(ckpt_prefix)
        else:
            print("Checkpoint is set to None. Could not load checkpoint.")  

    def setup_checkpoints(self) -> ():

        if self.model is not None:
            self.checkpoint = tf.train.Checkpoint(optimizer=self.optimizer, model=self.model) 
        else:
            print("Model is set to None. Could not instantiate checkpoint.")  

    def reinstantiate_model(self) -> ():

        self.model = LinderFitnessModel()
        self.setup_checkpoints()

    def train_step(self, x : tf.Tensor, y_true : tf.Tensor) -> (tf.Tensor, tf.Tensor):

        '''
        @param x:                   Training inputs. Expected dimensions are (NUM_SEQ, SEQ_LENGTH, NUM_CHARS)
        @param y:                   Training outputs. Expected dimensions are (NUM_SEQ,)
        @return training loss:      The training loss
        @return y_predict:          The predicted output of the model
        '''
        
        with tf.GradientTape() as tape:
            y_predict = self.model(x, is_training=True)
            loss = self.loss_fn(y_true, y_predict)
            # l2_losses = model.get_l2_loss(loss.shape)
            # loss = tf.math.add(loss, 1e-5*l2_losses)

        # apply gradient
        dLoss = tape.gradient(loss, self.model.trainable_variables)
        self.optimizer.apply_gradients(zip(dLoss, self.model.trainable_variables))

        # loss
        training_loss = tf.reduce_mean(loss)

        return training_loss, y_predict
    
    def train_model(self, Xcv : tf.Tensor, Ycv : tf.Tensor, cv_folds : int=10, print_on : bool=False) -> ():

        '''
        @param Xcv:         Training CV inputs. Expected dimensions are (NUM_SEQ, SEQ_LENGTH, NUM_CHARS)
        @param Ycv:         Training CV outputs. Expected dimensions are (NUM_SEQ,)
        @param cv_folds:    The number of cross validations folds.
        @param print_on:    A flag indicating if the function should print training and validation results.
        '''

        self.val_losses, self.val_metrics = [], []

        # self.reinstantiate_model()

        for cv in range(cv_folds):

            if print_on:
                print("--- CV FOLD", cv+1, "---\n")
            
            self.reinstantiate_model()

            Xtrain, Xval, Ytrain, Yval = train_test_split(Xcv, Ycv, test_size=0.05, random_state=42)

            cv_ckpt_files = []
            training_losses, training_metrics = [], []

            for i in range(self.num_epochs):

                batch_count = 0
                while batch_count < Xtrain.shape[0]:

                    iStart = batch_count
                    iEnd = min(batch_count + self.batch_size, Xtrain.shape[0])
                    x_batch_train = Xtrain[iStart:iEnd,:,:]
                    y_batch_train = Ytrain[iStart:iEnd]
                    training_loss, y_predict = self.train_step(x_batch_train, y_batch_train)
                    batch_count += self.batch_size
                
                training_losses.append(training_loss)
                r2 = self.metric(y_batch_train, y_predict)
                training_metrics.append(r2)

                # save checkpoint
                suffix = "_cv" + str(cv+1)
                ckpt_path = self.save_model(suffix)
                cv_ckpt_files.append(ckpt_path)

                if print_on and (i+1) % 10 == 0:
                    print("Epoch", i+1, ", loss =", training_loss.numpy(), ", r2 =", r2.numpy()) 

            # find and load the best training ckpt for this fold 
            cv_best_ckpt = self.find_best_checkpoint(training_losses, cv_ckpt_files)
            self.cv_best_ckpts.append(cv_best_ckpt)
            self.load_model(cv_best_ckpt)

            # run validation set   
            Yval_predict = self.model(Xval, is_training=False)
            val_loss = self.loss_fn(Yval, Yval_predict)
            val_r2 = self.metric(Yval, Yval_predict)
            self.val_losses.append(tf.reduce_mean(val_loss))
            self.val_metrics.append(val_r2)

            if print_on:
                print("\n------------------------------------")
                print("VAL results: loss =", val_loss.numpy(), ", r2 =", val_r2.numpy())
                print("------------------------------------\n")
        
        # store the best performing checkpoint based on validation
        self.best_ckpt = self.find_best_checkpoint(self.val_losses, self.cv_best_ckpts)
        print("\nBest checkpoint:", self.best_ckpt)
        
        self.is_trained = True
    
    def find_best_checkpoint(self, losses : list, ckpt_files : list) -> (str):

        '''
        @param losses:          A list of loss values of length N.
        @param ckpt_files:      A list of checkpoints filenames of length N.
        @return best_ckpt_file: The checkpoint file for the CV fold with the lowest validation loss.
        '''

        if len(losses) == 0 or len(ckpt_files) == 0 or len(losses) != len(ckpt_files):
            return ""

        iBest = losses.index(min(losses))
        if len(ckpt_files) > 0 and iBest < len(ckpt_files):
            return ckpt_files[iBest]
        else:
            return ckpt_files[-1]
    
    def test(self, Xtest : tf.Tensor, Ytest : tf.Tensor, print_on : bool=False) -> (float):

        '''
        @param Xtest:           Testing input samples.
        @param Ytest:           Testing actual outputs.
        @param print_on:        A flag indicating if the function should print training and validation results.
        @return test_2:         The R2 score of the test samples.
        '''

        if not self.is_trained:
            print("Model is not trained. Please run train_model() first.")
            return        
        
        self.reinstantiate_model()
        self.load_model(self.best_ckpt)
    
        Ytest_predict = self.model(Xtest, is_training=False)
        test_loss = self.loss_fn(Ytest, Ytest_predict)
        test_r2 = self.metric(Ytest, Ytest_predict)

        if print_on:
            print("\n------------------------------------")
            print("TEST results: loss =", test_loss.numpy(), ", r2 =", test_r2.numpy())
            print("------------------------------------\n")

        return test_r2

    def get_trained_model(self) -> (LinderFitnessModel):

        '''
        @return model:              Returns the trained version of the model.
        '''

        if not self.is_trained:
            print("Model is not trained. Please run train_model() first.")
            return None       
        
        self.reinstantiate_model()
        self.load_model(self.best_ckpt)
        return self.model
    
    def get_best_results(self) -> (float):

        '''
        @return best_r2:            Returns the highest validation R2 score.
        '''

        if not self.is_trained:
            print("Model is not trained. Please run train_model() first.")
            return -1
        
        return max(self.val_metrics)

class ProPWMGenerator:

    def __init__(self, fitness_model : LinderFitnessModel, max_length: int, chars : list, \
        wt_seq : str="VDGV", random_seed : int=5) -> ():

        '''
        @param max_length:          The maximum length of the sequences.
        @param chars:               A list containing all possible characters (amino acids).
        @random_seed:               A seed used to init the python library random. Use this for 
                                    repeatable results. Otherwise, use None.
        '''

        if max_length < 0 or len(chars) == 0:
            print("Invalid input arguments. Please re-instantiate.")
            return

        self.num_chars = len(chars)
        self.seq_length = max_length
        self.pwm = np.zeros((self.seq_length, self.num_chars))
        self.existing_seqs = []
        self.is_trained = False
        self.char_idx = { chars[i] : i for i in range(len(chars))}
        self.idx_char = { i : chars[i] for i in range(len(chars))}
        random.seed(random_seed)      # initialize random seed for consistent results with each run

        if len(wt_seq) == 0:
            wt_seq = "VDGV"

        self.fitness_model = fitness_model
        
    def train(self, dataset : dict) -> ():

        '''
        @param dataset:         A dictionary of format { sequence(str) : fitness(float) }
        '''

        if type(dataset) != dict:
            print("Input to training must be a dictionary of format { sequence(str) : fitness(float) }")
            return 0
        
        if len(dataset) == 0:
            print("Dataset must be greater than zero")
            return 0
        
        # set self.existing_seqs so we don't duplicate them during generate()
        x, y = aa_encoding(dataset, self.seq_length, self.char_idx)
        self.existing_seqs = list(dataset.keys())

        # get character counts
        num_samples = x.shape[0]
        for i in range(num_samples):
            self.pwm += y[i] * x[i,:,:]
        
        # calculate probabilities
        sum_rows = np.sum(self.pwm, axis=1)
        for r in range(self.seq_length):
            self.pwm[r,:] /= sum_rows[r]
        
        self.is_trained = True
        
    def generate(self, max_iters : int=1000) -> (str, float):

        '''
        @param max_iters:           The maximum number of iterations allowed to generate one
                                    novel sequence
        @return novel_seq:          The novel sequence generated.
        @return fitness             The predicted fitness of the novel sequence.
        '''

        if not self.is_trained:
            print("Generator is not yet init or trained")
            return "", 0.0

        if max_iters <= 0:
            max_iters = 1000

        # start by find highest probability AA for each position
        high_prob_aa_idxs = np.argmax(self.pwm, axis=1)
        novel_seq = ""
        for idx in high_prob_aa_idxs:
            novel_seq += self.idx_char[idx]
        
        # check if this sequence already exists
        # if not, return sequence
        if novel_seq not in self.existing_seqs:
            self.existing_seqs.append(novel_seq)
            x_novel_seq, _ = aa_encoding({novel_seq : 0}, self.seq_length, self.char_idx)
            fitness = self.fitness_model(x_novel_seq).numpy()[0]
            return novel_seq, fitness
        
        # if sequence exists, iteratively replace a random char with an unused
        # high-probability AA until a new sequence has been found
        count = 0
        highest_prob_seq = novel_seq
        backup_seq = novel_seq
        num_mods = 1
        while novel_seq in self.existing_seqs and count < max_iters:

            # randomly replace one locus with a * to mark site of new AA modification
            novel_seq, pos_idxs = self.modify_sequence(novel_seq, num_mods=num_mods)
            if len(pos_idxs) == 0 or len(novel_seq) == 0:
                novel_seq = backup_seq
                count += 1
                continue
            
            # for each modified position, find a high-probability replacement AA
            for pos_idx in pos_idxs:

                # from matches, find the set of unused AA
                unused_chars = self.get_unused_chars(novel_seq)
                if len(unused_chars) == 0:
                    print("Could not find a matching sequence")
                    x_novel_seq, _ = aa_encoding({highest_prob_seq : 0}, self.seq_length, self.char_idx)
                    fitness = self.fitness_model(x_novel_seq).numpy()[0]
                    return highest_prob_seq, fitness
                
                # find the next highest unused AA in the PWM matrix
                ch_best = ""
                ch_prob = 0.0
                for ch in unused_chars:
                    ch_idx = self.char_idx[ch]
                    prob = self.pwm[pos_idx, ch_idx]
                    if prob > ch_prob:
                        ch_prob = prob
                        ch_best = ch

                # assign the next highest probability unused char to the novel sequence
                novel_seq = novel_seq[:pos_idx] + ch_best + novel_seq[pos_idx+1:]

            # update iterative variables
            backup_seq = novel_seq
            count += 1      
            if count % 10 == 0 and num_mods < self.seq_length-2:
                num_mods += 1

        # double check that our new sequence is unique
        if novel_seq in self.existing_seqs:
            print("Could not find a unique sequence")
            x_novel_seq, _ = aa_encoding({backup_seq : 0}, self.seq_length, self.char_idx)
            fitness = self.fitness_model(x_novel_seq).numpy()[0]
            return backup_seq, fitness

        # add novel_seq to existing sequences
        self.existing_seqs.append(novel_seq)
        x_novel_seq, _ = aa_encoding({novel_seq : 0}, self.seq_length, self.char_idx)
        fitness = self.fitness_model(x_novel_seq).numpy()[0]

        return novel_seq, fitness

    def modify_sequence(self, query : str, mod_char : str="*", num_mods : int=1) -> (str, list):

        '''
        @param query:           A query sequence of length self.seq_length
        @param mod_char:        The character used to identify which locus is to be modified
        @param num_mods:        The number of characters to be modified.
        @return mod_query       The modified query
        @return pos_idxs        The position indices which have been modified
        '''

        if len(query) != self.seq_length:
            return "", []
        
        if len(mod_char) == 0:
            mod_char = "*"
        
        if num_mods <= 0:
            num_mods = 1

        pos_idxs = []
        i = 0
        while i < num_mods:
            pos_idx = random.randint(0, self.seq_length-1)
            if pos_idx in pos_idxs:
                continue
            query = query[:pos_idx] + mod_char + query[pos_idx+1:]
            pos_idxs.append(pos_idx)
            i += 1

        return query, pos_idxs

    def get_unused_chars(self, query : str) -> (list):

        '''
        @param query:           A query sequence of length self.seq_length
        @return unused_chars:   A list of amino acids that have been unused in the current 
                                set of query matches (where * are represent any amino acid)
        '''

        if len(query) != self.seq_length or query == "*"*self.seq_length:
            return []

        unused_chars = list(self.char_idx.keys())
        # replace_idxs = [i for i, ch in enumerate(query) if ch == "*"]
        for s in self.existing_seqs:
            duplicate_chars = []
            is_match = True
            for i in range(self.seq_length):
                if query[i] == "*": 
                    duplicate_chars.append(s[i])
                elif query[i] != s[i]:
                    is_match = False
                    break
            if is_match:
                for dc in duplicate_chars:
                    if dc in unused_chars:
                        unused_chars.remove(dc)
        
        return unused_chars


# a function to test the LinderTrainer and LinderFitenssModel classes
def run_test_LinderTrainer() -> ():
    
    seqs = read_sequences("DataSet for Assignment.xlsx - Sheet1.csv")
    seq_length = len(list(seqs.keys())[0])
    aa_idx = { amino_acids[i] : i for i in range(len(amino_acids))}
    # idx_aa = { i : amino_acids[i] for i in range(len(amino_acids))}
    X, Y = aa_encoding(seqs, seq_length, aa_idx)
    Xcv, Xtest, Ycv, Ytest = train_test_split(X, Y, test_size=0.2, random_state=42)

    model_trainer = LinderTrainer()
    model_trainer.train_model(Xcv, Ycv, print_on=True)
    model_trainer.test(Xtest, Ytest, print_on=True)


if __name__ == "__main__":

    run_test_LinderTrainer()