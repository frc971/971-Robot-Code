function writeMatHeader(fd, number_of_states, number_of_outputs)
    fprintf(fd, 'typedef StateFeedbackLoop<%d, %d> MatrixClass;\n', number_of_states, number_of_outputs);
    fprintf(fd, '#define MATRIX_INIT ');
end