import numpy as np

def aa_encoding(dataset : dict, seq_length : int, char_idx : dict) -> (np.array, np.array):
    
    X = np.zeros((len(dataset), seq_length, len(char_idx)))
    Y = np.zeros((len(dataset),))

    #One-hot encoding
    for i, seq in enumerate(dataset):
        for j, aa in enumerate(seq):
            X[i, j, char_idx[aa]] = 1
        Y[i] = dataset[seq]

    return X, Y

def aa_decoding(self, onehot_encodings : np.array, seq_length : int, idx_char : dict) -> (list):
    
    if onehot_encodings.ndim != 3:
        print("Input must have 3 dimensions: (NUM_SAMPLES x SEQ_LENGTH x NUM_CHARS)")
        return []

    num_seqs = onehot_encodings.shape[0]
    seq_length = onehot_encodings.shape[1]

    decoded_aas = []

    for i in range(num_seqs):
        seq = onehot_encodings[i,:,:]
        seq_str = ""
        for j in range(seq_length):

            # check that there is only one 1 per row
            if np.sum(onehot_encodings[i,j,:]) != 1:
                print("There can only be one 1-value per row/column")
                return []
            
            idx = np.argmax(onehot_encodings[i,j,:])
            seq_str += idx_char[idx]
        
        decoded_aas.append(seq_str)

    return decoded_aas