from models import read_sequences

def count_matches(query_seqs : list, database_seqs : list) -> (int):

    '''
    @param query_seqs:          A list of query sequences
    @param database_seqs:       A list of the sequences from the dataset
    @return match_counts:       The number of query matches found in the dataset
    '''

    seq_length = len(database_seqs[0])
    match_counts = 0

    for q in query_seqs:

        if len(q) != seq_length:
            continue

        for d in database_seqs:

            if len(d) != seq_length:
                continue

            is_match = True
            for i in range(seq_length):
                if q[i] == "*": 
                    continue
                elif q[i] != d[i]:
                    is_match = False
                    break
            if is_match:
                match_counts += 1
    
    return match_counts


if __name__ == "__main__":

    seqs = read_sequences("DataSet for Assignment.xlsx - Sheet1.csv")
    database_seqs = list(seqs.keys())

    query_seqs = [ \
        "W*LP",
        "W*LS",
        "W*CP",
        "W*CS",
        "W*LY",
        "Y*LP",
        "Y*LS",
        "Y*CP",
        "Y*CS",
        "Y*LH"    
    ]

    num_matches = count_matches(query_seqs, database_seqs)
    num_seqs = len(database_seqs)

    print("Percentage of high-fitness matches:", float(num_matches/num_seqs)*100)

