Dimensions_Keep = [2 5 6];
Dimension_New = [1 3 4];
boundaries = [1, 2, 3; -1, -2, -3];

Polytope = load("Maximal_Polytope_4.mat").Maximal_Polytope;
Polytope2 = load("Maximal_Polytope_4.mat").Maximal_Polytope;
A_new = zeros(size(Polytope.A, 1) + 2 * size(Dimension_New, 2), size(Polytope.A, 2) + size(Dimension_New, 2));
b_new = zeros(size(Polytope.b, 1) + 2 * size(Dimension_New, 2), 1);

for kk = 1:size(Dimensions_Keep, 2)
    A_new(1:size(Polytope.A, 1), Dimensions_Keep(kk)) = Polytope.A(1:size(Polytope.A, 1), kk);
    b_new(1:size(Polytope.b, 1)) = Polytope.b;
end
for jj = 1:size(Dimension_New, 2)
    A_new(size(Polytope.A, 1) + 1 + 2 * (jj - 1):size(Polytope.A, 1) + 1 + 2 * (jj - 1) + 1, Dimension_New(jj)) = [1; -1];
    b_new(size(Polytope.b, 1) + 1 + 2 * (jj - 1):size(Polytope.b, 1) + 1 + 2 * (jj - 1) + 1) = [boundaries(1, jj), -boundaries(2, jj)];
end
res = polytope(A_new, b_new);
