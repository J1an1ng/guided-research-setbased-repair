classdef TestUnnestDict < matlab.unittest.TestCase
    methods (Test)

        function test(testCase)
            d = dictionary(uint32.empty, uint32.empty);
            d = d.insert(3, 4);
            d = d.insert(2, 3);
            d = d.insert(4, 5);
            d = d.insert(1, 2);
            d = d.insert(9, 11);
            d = d.insert(6, 3);
            d = d.insert(7, 10);
            d = d.insert(11, 12);
            d = d.insert(7, 10);
            d = d.insert(13, 11);
            d = d.insert(8, 10);
            d = d.insert(20, 13);

            exp = dictionary(uint32([1 6 7 8 9 20]), uint32([5 5 10 10 12 12]));
            act = unnest_dict(d);

            import matlab.unittest.TestCase
            import matlab.unittest.constraints.IsEqualTo
            import matlab.unittest.constraints.DictionaryComparator
            import matlab.unittest.constraints.NumericComparator

            testCase.verifyThat(act, IsEqualTo(exp, Using = DictionaryComparator(NumericComparator)));
        end

    end
end
