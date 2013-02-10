function writeMatFlash(fd, matrix, name)
    %fprintf(fd, '%s = init_matrix(%d, %d);\n', name, size(matrix, 1), size(matrix, 2));
    fprintf(fd, 'flash_matrix(%s, ', name);
    first_loop = 1;
    for i=1:size(matrix, 1)
        for j=1:size(matrix, 2)
            if first_loop
                first_loop = 0;
            else
                fprintf(fd, ', ');
            end
            fprintf(fd, '%.10f', matrix(i, j));
        end
    end
    fprintf(fd, ');\n');
end
