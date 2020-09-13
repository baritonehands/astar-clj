(ns baritonehands.astar-test
  (:require [clojure.test :refer :all]
            [baritonehands.astar :refer :all]))

(def floor1-cases
  [[[0 1] [0 4 5 1]]
   [[0 2] [0 4 5 1 2]]
   [[0 3] [0 4 5 1 2 3]]
   [[0 4] [0 4]]
   [[0 5] [0 4 5]]
   [[0 6] [0 4 5 6]]
   [[0 7] [0 4 5 1 2 3 7]]
   [[0 8] [0 4 5 6 10 9 8]]
   [[0 9] [0 4 5 6 10 9]]
   [[0 10] [0 4 5 6 10]]
   [[0 11] [0 4 5 6 10 11]]
   [[0 12] [0 4 5 6 10 9 8 12]]
   [[0 13] [0 4 5 6 10 9 13]]
   [[0 14] [0 4 5 6 10 14]]
   [[0 15] [0 4 5 6 10 14 15]]
   [[1 0] [1 5 4 0]]
   [[1 2] [1 2]]
   [[1 3] [1 2 3]]
   [[1 4] [1 5 4]]
   [[1 5] [1 5]]
   [[1 6] [1 5 6]]
   [[1 7] [1 2 3 7]]
   [[1 8] [1 5 6 10 9 8]]
   [[1 9] [1 5 6 10 9]]
   [[1 10] [1 5 6 10]]
   [[1 11] [1 2 3 7 11]]
   [[1 12] [1 5 6 10 9 8 12]]
   [[1 13] [1 5 6 10 9 13]]
   [[1 14] [1 5 6 10 14]]
   [[1 15] [1 5 6 10 14 15]]
   [[2 0] [2 1 5 4 0]]
   [[2 1] [2 1]]
   [[2 3] [2 3]]
   [[2 4] [2 1 5 4]]
   [[2 5] [2 1 5]]
   [[2 6] [2 1 5 6]]
   [[2 7] [2 3 7]]
   [[2 8] [2 3 7 11 10 9 8]]
   [[2 9] [2 3 7 11 10 9]]
   [[2 10] [2 3 7 11 10]]
   [[2 11] [2 3 7 11]]
   [[2 12] [2 3 7 11 10 9 8 12]]
   [[2 13] [2 3 7 11 10 9 13]]
   [[2 14] [2 3 7 11 10 14]]
   [[2 15] [2 3 7 11 10 14 15]]
   [[3 0] [3 2 1 5 4 0]]
   [[3 1] [3 2 1]]
   [[3 2] [3 2]]
   [[3 4] [3 2 1 5 4]]
   [[3 5] [3 2 1 5]]
   [[3 6] [3 7 11 10 6]]
   [[3 7] [3 7]]
   [[3 8] [3 7 11 10 9 8]]
   [[3 9] [3 7 11 10 9]]
   [[3 10] [3 7 11 10]]
   [[3 11] [3 7 11]]
   [[3 12] [3 7 11 10 9 8 12]]
   [[3 13] [3 7 11 10 9 13]]
   [[3 14] [3 7 11 10 14]]
   [[3 15] [3 7 11 10 14 15]]
   [[4 0] [4 0]]
   [[4 1] [4 5 1]]
   [[4 2] [4 5 1 2]]
   [[4 3] [4 5 1 2 3]]
   [[4 5] [4 5]]
   [[4 6] [4 5 6]]
   [[4 7] [4 5 1 2 3 7]]
   [[4 8] [4 5 6 10 9 8]]
   [[4 9] [4 5 6 10 9]]
   [[4 10] [4 5 6 10]]
   [[4 11] [4 5 6 10 11]]
   [[4 12] [4 5 6 10 9 8 12]]
   [[4 13] [4 5 6 10 9 13]]
   [[4 14] [4 5 6 10 14]]
   [[4 15] [4 5 6 10 14 15]]
   [[5 0] [5 4 0]]
   [[5 1] [5 1]]
   [[5 2] [5 1 2]]
   [[5 3] [5 1 2 3]]
   [[5 4] [5 4]]
   [[5 6] [5 6]]
   [[5 7] [5 1 2 3 7]]
   [[5 8] [5 6 10 9 8]]
   [[5 9] [5 6 10 9]]
   [[5 10] [5 6 10]]
   [[5 11] [5 6 10 11]]
   [[5 12] [5 6 10 9 8 12]]
   [[5 13] [5 6 10 9 13]]
   [[5 14] [5 6 10 14]]
   [[5 15] [5 6 10 14 15]]
   [[6 0] [6 5 4 0]]
   [[6 1] [6 5 1]]
   [[6 2] [6 5 1 2]]
   [[6 3] [6 10 11 7 3]]
   [[6 4] [6 5 4]]
   [[6 5] [6 5]]
   [[6 7] [6 10 11 7]]
   [[6 8] [6 10 9 8]]
   [[6 9] [6 10 9]]
   [[6 10] [6 10]]
   [[6 11] [6 10 11]]
   [[6 12] [6 10 9 8 12]]
   [[6 13] [6 10 9 13]]
   [[6 14] [6 10 14]]
   [[6 15] [6 10 14 15]]
   [[7 0] [7 3 2 1 5 4 0]]
   [[7 1] [7 3 2 1]]
   [[7 2] [7 3 2]]
   [[7 3] [7 3]]
   [[7 4] [7 3 2 1 5 4]]
   [[7 5] [7 3 2 1 5]]
   [[7 6] [7 11 10 6]]
   [[7 8] [7 11 10 9 8]]
   [[7 9] [7 11 10 9]]
   [[7 10] [7 11 10]]
   [[7 11] [7 11]]
   [[7 12] [7 11 10 9 8 12]]
   [[7 13] [7 11 10 9 13]]
   [[7 14] [7 11 10 14]]
   [[7 15] [7 11 10 14 15]]
   [[8 0] [8 9 10 6 5 4 0]]
   [[8 1] [8 9 10 6 5 1]]
   [[8 2] [8 9 10 6 5 1 2]]
   [[8 3] [8 9 10 11 7 3]]
   [[8 4] [8 9 10 6 5 4]]
   [[8 5] [8 9 10 6 5]]
   [[8 6] [8 9 10 6]]
   [[8 7] [8 9 10 11 7]]
   [[8 9] [8 9]]
   [[8 10] [8 9 10]]
   [[8 11] [8 9 10 11]]
   [[8 12] [8 12]]
   [[8 13] [8 9 13]]
   [[8 14] [8 9 10 14]]
   [[8 15] [8 9 10 14 15]]
   [[9 0] [9 10 6 5 4 0]]
   [[9 1] [9 10 6 5 1]]
   [[9 2] [9 10 6 5 1 2]]
   [[9 3] [9 10 11 7 3]]
   [[9 4] [9 10 6 5 4]]
   [[9 5] [9 10 6 5]]
   [[9 6] [9 10 6]]
   [[9 7] [9 10 11 7]]
   [[9 8] [9 8]]
   [[9 10] [9 10]]
   [[9 11] [9 10 11]]
   [[9 12] [9 8 12]]
   [[9 13] [9 13]]
   [[9 14] [9 10 14]]
   [[9 15] [9 10 14 15]]
   [[10 0] [10 6 5 4 0]]
   [[10 1] [10 6 5 1]]
   [[10 2] [10 6 5 1 2]]
   [[10 3] [10 11 7 3]]
   [[10 4] [10 6 5 4]]
   [[10 5] [10 6 5]]
   [[10 6] [10 6]]
   [[10 7] [10 11 7]]
   [[10 8] [10 9 8]]
   [[10 9] [10 9]]
   [[10 11] [10 11]]
   [[10 12] [10 9 8 12]]
   [[10 13] [10 9 13]]
   [[10 14] [10 14]]
   [[10 15] [10 14 15]]
   [[11 0] [11 10 6 5 4 0]]
   [[11 1] [11 7 3 2 1]]
   [[11 2] [11 7 3 2]]
   [[11 3] [11 7 3]]
   [[11 4] [11 10 6 5 4]]
   [[11 5] [11 10 6 5]]
   [[11 6] [11 10 6]]
   [[11 7] [11 7]]
   [[11 8] [11 10 9 8]]
   [[11 9] [11 10 9]]
   [[11 10] [11 10]]
   [[11 12] [11 10 9 8 12]]
   [[11 13] [11 10 9 13]]
   [[11 14] [11 10 14]]
   [[11 15] [11 10 14 15]]
   [[12 0] [12 8 9 10 6 5 4 0]]
   [[12 1] [12 8 9 10 6 5 1]]
   [[12 2] [12 8 9 10 6 5 1 2]]
   [[12 3] [12 8 9 10 11 7 3]]
   [[12 4] [12 8 9 10 6 5 4]]
   [[12 5] [12 8 9 10 6 5]]
   [[12 6] [12 8 9 10 6]]
   [[12 7] [12 8 9 10 11 7]]
   [[12 8] [12 8]]
   [[12 9] [12 8 9]]
   [[12 10] [12 8 9 10]]
   [[12 11] [12 8 9 10 11]]
   [[12 13] [12 8 9 13]]
   [[12 14] [12 8 9 10 14]]
   [[12 15] [12 8 9 10 14 15]]
   [[13 0] [13 9 10 6 5 4 0]]
   [[13 1] [13 9 10 6 5 1]]
   [[13 2] [13 9 10 6 5 1 2]]
   [[13 3] [13 9 10 11 7 3]]
   [[13 4] [13 9 10 6 5 4]]
   [[13 5] [13 9 10 6 5]]
   [[13 6] [13 9 10 6]]
   [[13 7] [13 9 10 11 7]]
   [[13 8] [13 9 8]]
   [[13 9] [13 9]]
   [[13 10] [13 9 10]]
   [[13 11] [13 9 10 11]]
   [[13 12] [13 9 8 12]]
   [[13 14] [13 9 10 14]]
   [[13 15] [13 9 10 14 15]]
   [[14 0] [14 10 6 5 4 0]]
   [[14 1] [14 10 6 5 1]]
   [[14 2] [14 10 6 5 1 2]]
   [[14 3] [14 10 11 7 3]]
   [[14 4] [14 10 6 5 4]]
   [[14 5] [14 10 6 5]]
   [[14 6] [14 10 6]]
   [[14 7] [14 10 11 7]]
   [[14 8] [14 10 9 8]]
   [[14 9] [14 10 9]]
   [[14 10] [14 10]]
   [[14 11] [14 10 11]]
   [[14 12] [14 10 9 8 12]]
   [[14 13] [14 10 9 13]]
   [[14 15] [14 15]]
   [[15 0] [15 14 10 6 5 4 0]]
   [[15 1] [15 14 10 6 5 1]]
   [[15 2] [15 14 10 6 5 1 2]]
   [[15 3] [15 14 10 11 7 3]]
   [[15 4] [15 14 10 6 5 4]]
   [[15 5] [15 14 10 6 5]]
   [[15 6] [15 14 10 6]]
   [[15 7] [15 14 10 11 7]]
   [[15 8] [15 14 10 9 8]]
   [[15 9] [15 14 10 9]]
   [[15 10] [15 14 10]]
   [[15 11] [15 14 10 11]]
   [[15 12] [15 14 10 9 8 12]]
   [[15 13] [15 14 10 9 13]]
   [[15 14] [15 14]]])

(def floor2-cases
  [[[0 1] [0 4 5 1]]
   [[0 2] [0 4 5 6 2]]
   [[0 3] [0 4 5 6 7 3]]
   [[0 4] [0 4]]
   [[0 5] [0 4 5]]
   [[0 6] [0 4 5 6]]
   [[0 7] [0 4 5 6 7]]
   [[0 8] [0 4 8]]
   [[0 9] [0 4 8 9]]
   [[0 10] [0 4 8 9 10]]
   [[0 11] [0 4 5 6 7 11]]
   [[0 12] [0 4 8 12]]
   [[0 13] [0 4 8 9 13]]
   [[0 14] [0 4 8 9 10 14]]
   [[0 15] [0 4 5 6 7 11 15]]
   [[1 0] [1 5 4 0]]
   [[1 2] [1 5 6 2]]
   [[1 3] [1 5 6 7 3]]
   [[1 4] [1 5 4]]
   [[1 5] [1 5]]
   [[1 6] [1 5 6]]
   [[1 7] [1 5 6 7]]
   [[1 8] [1 5 4 8]]
   [[1 9] [1 5 4 8 9]]
   [[1 10] [1 5 6 7 11 10]]
   [[1 11] [1 5 6 7 11]]
   [[1 12] [1 5 4 8 12]]
   [[1 13] [1 5 4 8 9 13]]
   [[1 14] [1 5 6 7 11 10 14]]
   [[1 15] [1 5 6 7 11 15]]
   [[2 0] [2 6 5 4 0]]
   [[2 1] [2 6 5 1]]
   [[2 3] [2 6 7 3]]
   [[2 4] [2 6 5 4]]
   [[2 5] [2 6 5]]
   [[2 6] [2 6]]
   [[2 7] [2 6 7]]
   [[2 8] [2 6 5 4 8]]
   [[2 9] [2 6 7 11 10 9]]
   [[2 10] [2 6 7 11 10]]
   [[2 11] [2 6 7 11]]
   [[2 12] [2 6 5 4 8 12]]
   [[2 13] [2 6 7 11 10 9 13]]
   [[2 14] [2 6 7 11 10 14]]
   [[2 15] [2 6 7 11 15]]
   [[3 0] [3 7 6 5 4 0]]
   [[3 1] [3 7 6 5 1]]
   [[3 2] [3 7 6 2]]
   [[3 4] [3 7 6 5 4]]
   [[3 5] [3 7 6 5]]
   [[3 6] [3 7 6]]
   [[3 7] [3 7]]
   [[3 8] [3 7 11 10 9 8]]
   [[3 9] [3 7 11 10 9]]
   [[3 10] [3 7 11 10]]
   [[3 11] [3 7 11]]
   [[3 12] [3 7 11 10 9 8 12]]
   [[3 13] [3 7 11 10 9 13]]
   [[3 14] [3 7 11 10 14]]
   [[3 15] [3 7 11 15]]
   [[4 0] [4 0]]
   [[4 1] [4 5 1]]
   [[4 2] [4 5 6 2]]
   [[4 3] [4 5 6 7 3]]
   [[4 5] [4 5]]
   [[4 6] [4 5 6]]
   [[4 7] [4 5 6 7]]
   [[4 8] [4 8]]
   [[4 9] [4 8 9]]
   [[4 10] [4 8 9 10]]
   [[4 11] [4 5 6 7 11]]
   [[4 12] [4 8 12]]
   [[4 13] [4 8 9 13]]
   [[4 14] [4 8 9 10 14]]
   [[4 15] [4 5 6 7 11 15]]
   [[5 0] [5 4 0]]
   [[5 1] [5 1]]
   [[5 2] [5 6 2]]
   [[5 3] [5 6 7 3]]
   [[5 4] [5 4]]
   [[5 6] [5 6]]
   [[5 7] [5 6 7]]
   [[5 8] [5 4 8]]
   [[5 9] [5 4 8 9]]
   [[5 10] [5 6 7 11 10]]
   [[5 11] [5 6 7 11]]
   [[5 12] [5 4 8 12]]
   [[5 13] [5 4 8 9 13]]
   [[5 14] [5 6 7 11 10 14]]
   [[5 15] [5 6 7 11 15]]
   [[6 0] [6 5 4 0]]
   [[6 1] [6 5 1]]
   [[6 2] [6 2]]
   [[6 3] [6 7 3]]
   [[6 4] [6 5 4]]
   [[6 5] [6 5]]
   [[6 7] [6 7]]
   [[6 8] [6 5 4 8]]
   [[6 9] [6 7 11 10 9]]
   [[6 10] [6 7 11 10]]
   [[6 11] [6 7 11]]
   [[6 12] [6 5 4 8 12]]
   [[6 13] [6 7 11 10 9 13]]
   [[6 14] [6 7 11 10 14]]
   [[6 15] [6 7 11 15]]
   [[7 0] [7 6 5 4 0]]
   [[7 1] [7 6 5 1]]
   [[7 2] [7 6 2]]
   [[7 3] [7 3]]
   [[7 4] [7 6 5 4]]
   [[7 5] [7 6 5]]
   [[7 6] [7 6]]
   [[7 8] [7 11 10 9 8]]
   [[7 9] [7 11 10 9]]
   [[7 10] [7 11 10]]
   [[7 11] [7 11]]
   [[7 12] [7 11 10 9 8 12]]
   [[7 13] [7 11 10 9 13]]
   [[7 14] [7 11 10 14]]
   [[7 15] [7 11 15]]
   [[8 0] [8 4 0]]
   [[8 1] [8 4 5 1]]
   [[8 2] [8 4 5 6 2]]
   [[8 3] [8 4 5 6 7 3]]
   [[8 4] [8 4]]
   [[8 5] [8 4 5]]
   [[8 6] [8 4 5 6]]
   [[8 7] [8 4 5 6 7]]
   [[8 9] [8 9]]
   [[8 10] [8 9 10]]
   [[8 11] [8 9 10 11]]
   [[8 12] [8 12]]
   [[8 13] [8 9 13]]
   [[8 14] [8 9 10 14]]
   [[8 15] [8 9 10 11 15]]
   [[9 0] [9 8 4 0]]
   [[9 1] [9 8 4 5 1]]
   [[9 2] [9 10 11 7 6 2]]
   [[9 3] [9 10 11 7 3]]
   [[9 4] [9 8 4]]
   [[9 5] [9 8 4 5]]
   [[9 6] [9 10 11 7 6]]
   [[9 7] [9 10 11 7]]
   [[9 8] [9 8]]
   [[9 10] [9 10]]
   [[9 11] [9 10 11]]
   [[9 12] [9 8 12]]
   [[9 13] [9 13]]
   [[9 14] [9 10 14]]
   [[9 15] [9 10 11 15]]
   [[10 0] [10 9 8 4 0]]
   [[10 1] [10 11 7 6 5 1]]
   [[10 2] [10 11 7 6 2]]
   [[10 3] [10 11 7 3]]
   [[10 4] [10 9 8 4]]
   [[10 5] [10 11 7 6 5]]
   [[10 6] [10 11 7 6]]
   [[10 7] [10 11 7]]
   [[10 8] [10 9 8]]
   [[10 9] [10 9]]
   [[10 11] [10 11]]
   [[10 12] [10 9 8 12]]
   [[10 13] [10 9 13]]
   [[10 14] [10 14]]
   [[10 15] [10 11 15]]
   [[11 0] [11 7 6 5 4 0]]
   [[11 1] [11 7 6 5 1]]
   [[11 2] [11 7 6 2]]
   [[11 3] [11 7 3]]
   [[11 4] [11 7 6 5 4]]
   [[11 5] [11 7 6 5]]
   [[11 6] [11 7 6]]
   [[11 7] [11 7]]
   [[11 8] [11 10 9 8]]
   [[11 9] [11 10 9]]
   [[11 10] [11 10]]
   [[11 12] [11 10 9 8 12]]
   [[11 13] [11 10 9 13]]
   [[11 14] [11 10 14]]
   [[11 15] [11 15]]
   [[12 0] [12 8 4 0]]
   [[12 1] [12 8 4 5 1]]
   [[12 2] [12 8 4 5 6 2]]
   [[12 3] [12 8 4 5 6 7 3]]
   [[12 4] [12 8 4]]
   [[12 5] [12 8 4 5]]
   [[12 6] [12 8 4 5 6]]
   [[12 7] [12 8 4 5 6 7]]
   [[12 8] [12 8]]
   [[12 9] [12 8 9]]
   [[12 10] [12 8 9 10]]
   [[12 11] [12 8 9 10 11]]
   [[12 13] [12 8 9 13]]
   [[12 14] [12 8 9 10 14]]
   [[12 15] [12 8 9 10 11 15]]
   [[13 0] [13 9 8 4 0]]
   [[13 1] [13 9 8 4 5 1]]
   [[13 2] [13 9 10 11 7 6 2]]
   [[13 3] [13 9 10 11 7 3]]
   [[13 4] [13 9 8 4]]
   [[13 5] [13 9 8 4 5]]
   [[13 6] [13 9 10 11 7 6]]
   [[13 7] [13 9 10 11 7]]
   [[13 8] [13 9 8]]
   [[13 9] [13 9]]
   [[13 10] [13 9 10]]
   [[13 11] [13 9 10 11]]
   [[13 12] [13 9 8 12]]
   [[13 14] [13 9 10 14]]
   [[13 15] [13 9 10 11 15]]
   [[14 0] [14 10 9 8 4 0]]
   [[14 1] [14 10 11 7 6 5 1]]
   [[14 2] [14 10 11 7 6 2]]
   [[14 3] [14 10 11 7 3]]
   [[14 4] [14 10 9 8 4]]
   [[14 5] [14 10 11 7 6 5]]
   [[14 6] [14 10 11 7 6]]
   [[14 7] [14 10 11 7]]
   [[14 8] [14 10 9 8]]
   [[14 9] [14 10 9]]
   [[14 10] [14 10]]
   [[14 11] [14 10 11]]
   [[14 12] [14 10 9 8 12]]
   [[14 13] [14 10 9 13]]
   [[14 15] [14 10 11 15]]
   [[15 0] [15 11 7 6 5 4 0]]
   [[15 1] [15 11 7 6 5 1]]
   [[15 2] [15 11 7 6 2]]
   [[15 3] [15 11 7 3]]
   [[15 4] [15 11 7 6 5 4]]
   [[15 5] [15 11 7 6 5]]
   [[15 6] [15 11 7 6]]
   [[15 7] [15 11 7]]
   [[15 8] [15 11 10 9 8]]
   [[15 9] [15 11 10 9]]
   [[15 10] [15 11 10]]
   [[15 11] [15 11]]
   [[15 12] [15 11 10 9 8 12]]
   [[15 13] [15 11 10 9 13]]
   [[15 14] [15 11 10 14]]])

(def floor3-cases
  [[[0 1] [0 1]]
   [[0 2] [0 1 2]]
   [[0 3] [0 1 2 3]]
   [[0 4] [0 4]]
   [[0 5] [0 1 5]]
   [[0 6] [0 1 5 6]]
   [[0 7] [0 1 2 3 7]]
   [[0 8] [0 1 5 9 13 12 8]]
   [[0 9] [0 1 5 9]]
   [[0 10] [0 1 2 3 7 11 10]]
   [[0 11] [0 1 2 3 7 11]]
   [[0 12] [0 1 5 9 13 12]]
   [[0 13] [0 1 5 9 13]]
   [[0 14] [0 1 5 9 13 14]]
   [[0 15] [0 1 2 3 7 11 15]]
   [[1 0] [1 0]]
   [[1 2] [1 2]]
   [[1 3] [1 2 3]]
   [[1 4] [1 0 4]]
   [[1 5] [1 5]]
   [[1 6] [1 5 6]]
   [[1 7] [1 2 3 7]]
   [[1 8] [1 5 9 13 12 8]]
   [[1 9] [1 5 9]]
   [[1 10] [1 2 3 7 11 10]]
   [[1 11] [1 2 3 7 11]]
   [[1 12] [1 5 9 13 12]]
   [[1 13] [1 5 9 13]]
   [[1 14] [1 5 9 13 14]]
   [[1 15] [1 2 3 7 11 15]]
   [[2 0] [2 1 0]]
   [[2 1] [2 1]]
   [[2 3] [2 3]]
   [[2 4] [2 1 0 4]]
   [[2 5] [2 1 5]]
   [[2 6] [2 1 5 6]]
   [[2 7] [2 3 7]]
   [[2 8] [2 1 5 9 13 12 8]]
   [[2 9] [2 1 5 9]]
   [[2 10] [2 3 7 11 10]]
   [[2 11] [2 3 7 11]]
   [[2 12] [2 1 5 9 13 12]]
   [[2 13] [2 1 5 9 13]]
   [[2 14] [2 1 5 9 13 14]]
   [[2 15] [2 3 7 11 15]]
   [[3 0] [3 2 1 0]]
   [[3 1] [3 2 1]]
   [[3 2] [3 2]]
   [[3 4] [3 2 1 0 4]]
   [[3 5] [3 2 1 5]]
   [[3 6] [3 2 1 5 6]]
   [[3 7] [3 7]]
   [[3 8] [3 2 1 5 9 13 12 8]]
   [[3 9] [3 2 1 5 9]]
   [[3 10] [3 7 11 10]]
   [[3 11] [3 7 11]]
   [[3 12] [3 2 1 5 9 13 12]]
   [[3 13] [3 2 1 5 9 13]]
   [[3 14] [3 7 11 10 14]]
   [[3 15] [3 7 11 15]]
   [[4 0] [4 0]]
   [[4 1] [4 0 1]]
   [[4 2] [4 0 1 2]]
   [[4 3] [4 0 1 2 3]]
   [[4 5] [4 0 1 5]]
   [[4 6] [4 0 1 5 6]]
   [[4 7] [4 0 1 2 3 7]]
   [[4 8] [4 0 1 5 9 13 12 8]]
   [[4 9] [4 0 1 5 9]]
   [[4 10] [4 0 1 2 3 7 11 10]]
   [[4 11] [4 0 1 2 3 7 11]]
   [[4 12] [4 0 1 5 9 13 12]]
   [[4 13] [4 0 1 5 9 13]]
   [[4 14] [4 0 1 5 9 13 14]]
   [[4 15] [4 0 1 2 3 7 11 15]]
   [[5 0] [5 1 0]]
   [[5 1] [5 1]]
   [[5 2] [5 1 2]]
   [[5 3] [5 1 2 3]]
   [[5 4] [5 1 0 4]]
   [[5 6] [5 6]]
   [[5 7] [5 1 2 3 7]]
   [[5 8] [5 9 13 12 8]]
   [[5 9] [5 9]]
   [[5 10] [5 9 13 14 10]]
   [[5 11] [5 1 2 3 7 11]]
   [[5 12] [5 9 13 12]]
   [[5 13] [5 9 13]]
   [[5 14] [5 9 13 14]]
   [[5 15] [5 1 2 3 7 11 15]]
   [[6 0] [6 5 1 0]]
   [[6 1] [6 5 1]]
   [[6 2] [6 5 1 2]]
   [[6 3] [6 5 1 2 3]]
   [[6 4] [6 5 1 0 4]]
   [[6 5] [6 5]]
   [[6 7] [6 5 1 2 3 7]]
   [[6 8] [6 5 9 13 12 8]]
   [[6 9] [6 5 9]]
   [[6 10] [6 5 9 13 14 10]]
   [[6 11] [6 5 1 2 3 7 11]]
   [[6 12] [6 5 9 13 12]]
   [[6 13] [6 5 9 13]]
   [[6 14] [6 5 9 13 14]]
   [[6 15] [6 5 1 2 3 7 11 15]]
   [[7 0] [7 3 2 1 0]]
   [[7 1] [7 3 2 1]]
   [[7 2] [7 3 2]]
   [[7 3] [7 3]]
   [[7 4] [7 3 2 1 0 4]]
   [[7 5] [7 3 2 1 5]]
   [[7 6] [7 3 2 1 5 6]]
   [[7 8] [7 11 10 14 13 12 8]]
   [[7 9] [7 11 10 14 13 9]]
   [[7 10] [7 11 10]]
   [[7 11] [7 11]]
   [[7 12] [7 11 10 14 13 12]]
   [[7 13] [7 11 10 14 13]]
   [[7 14] [7 11 10 14]]
   [[7 15] [7 11 15]]
   [[8 0] [8 12 13 9 5 1 0]]
   [[8 1] [8 12 13 9 5 1]]
   [[8 2] [8 12 13 9 5 1 2]]
   [[8 3] [8 12 13 9 5 1 2 3]]
   [[8 4] [8 12 13 9 5 1 0 4]]
   [[8 5] [8 12 13 9 5]]
   [[8 6] [8 12 13 9 5 6]]
   [[8 7] [8 12 13 14 10 11 7]]
   [[8 9] [8 12 13 9]]
   [[8 10] [8 12 13 14 10]]
   [[8 11] [8 12 13 14 10 11]]
   [[8 12] [8 12]]
   [[8 13] [8 12 13]]
   [[8 14] [8 12 13 14]]
   [[8 15] [8 12 13 14 10 11 15]]
   [[9 0] [9 5 1 0]]
   [[9 1] [9 5 1]]
   [[9 2] [9 5 1 2]]
   [[9 3] [9 5 1 2 3]]
   [[9 4] [9 5 1 0 4]]
   [[9 5] [9 5]]
   [[9 6] [9 5 6]]
   [[9 7] [9 5 1 2 3 7]]
   [[9 8] [9 13 12 8]]
   [[9 10] [9 13 14 10]]
   [[9 11] [9 13 14 10 11]]
   [[9 12] [9 13 12]]
   [[9 13] [9 13]]
   [[9 14] [9 13 14]]
   [[9 15] [9 13 14 10 11 15]]
   [[10 0] [10 11 7 3 2 1 0]]
   [[10 1] [10 11 7 3 2 1]]
   [[10 2] [10 11 7 3 2]]
   [[10 3] [10 11 7 3]]
   [[10 4] [10 14 13 9 5 1 0 4]]
   [[10 5] [10 14 13 9 5]]
   [[10 6] [10 14 13 9 5 6]]
   [[10 7] [10 11 7]]
   [[10 8] [10 14 13 12 8]]
   [[10 9] [10 14 13 9]]
   [[10 11] [10 11]]
   [[10 12] [10 14 13 12]]
   [[10 13] [10 14 13]]
   [[10 14] [10 14]]
   [[10 15] [10 11 15]]
   [[11 0] [11 7 3 2 1 0]]
   [[11 1] [11 7 3 2 1]]
   [[11 2] [11 7 3 2]]
   [[11 3] [11 7 3]]
   [[11 4] [11 7 3 2 1 0 4]]
   [[11 5] [11 7 3 2 1 5]]
   [[11 6] [11 7 3 2 1 5 6]]
   [[11 7] [11 7]]
   [[11 8] [11 10 14 13 12 8]]
   [[11 9] [11 10 14 13 9]]
   [[11 10] [11 10]]
   [[11 12] [11 10 14 13 12]]
   [[11 13] [11 10 14 13]]
   [[11 14] [11 10 14]]
   [[11 15] [11 15]]
   [[12 0] [12 13 9 5 1 0]]
   [[12 1] [12 13 9 5 1]]
   [[12 2] [12 13 9 5 1 2]]
   [[12 3] [12 13 9 5 1 2 3]]
   [[12 4] [12 13 9 5 1 0 4]]
   [[12 5] [12 13 9 5]]
   [[12 6] [12 13 9 5 6]]
   [[12 7] [12 13 14 10 11 7]]
   [[12 8] [12 8]]
   [[12 9] [12 13 9]]
   [[12 10] [12 13 14 10]]
   [[12 11] [12 13 14 10 11]]
   [[12 13] [12 13]]
   [[12 14] [12 13 14]]
   [[12 15] [12 13 14 10 11 15]]
   [[13 0] [13 9 5 1 0]]
   [[13 1] [13 9 5 1]]
   [[13 2] [13 9 5 1 2]]
   [[13 3] [13 9 5 1 2 3]]
   [[13 4] [13 9 5 1 0 4]]
   [[13 5] [13 9 5]]
   [[13 6] [13 9 5 6]]
   [[13 7] [13 14 10 11 7]]
   [[13 8] [13 12 8]]
   [[13 9] [13 9]]
   [[13 10] [13 14 10]]
   [[13 11] [13 14 10 11]]
   [[13 12] [13 12]]
   [[13 14] [13 14]]
   [[13 15] [13 14 10 11 15]]
   [[14 0] [14 13 9 5 1 0]]
   [[14 1] [14 13 9 5 1]]
   [[14 2] [14 10 11 7 3 2]]
   [[14 3] [14 10 11 7 3]]
   [[14 4] [14 13 9 5 1 0 4]]
   [[14 5] [14 13 9 5]]
   [[14 6] [14 13 9 5 6]]
   [[14 7] [14 10 11 7]]
   [[14 8] [14 13 12 8]]
   [[14 9] [14 13 9]]
   [[14 10] [14 10]]
   [[14 11] [14 10 11]]
   [[14 12] [14 13 12]]
   [[14 13] [14 13]]
   [[14 15] [14 10 11 15]]
   [[15 0] [15 11 7 3 2 1 0]]
   [[15 1] [15 11 7 3 2 1]]
   [[15 2] [15 11 7 3 2]]
   [[15 3] [15 11 7 3]]
   [[15 4] [15 11 7 3 2 1 0 4]]
   [[15 5] [15 11 7 3 2 1 5]]
   [[15 6] [15 11 7 3 2 1 5 6]]
   [[15 7] [15 11 7]]
   [[15 8] [15 11 10 14 13 12 8]]
   [[15 9] [15 11 10 14 13 9]]
   [[15 10] [15 11 10]]
   [[15 11] [15 11]]
   [[15 12] [15 11 10 14 13 12]]
   [[15 13] [15 11 10 14 13]]
   [[15 14] [15 11 10 14]]])

(defn run-one-floor [walls floor]
  (doseq [[[start end] path] floor]
    (is (= path (dfs walls start end))
        (str start " -> " end))))

(deftest floor1-test
  (run-one-floor (floor-walls 0) floor1-cases))

(deftest floor2-test
  (run-one-floor (floor-walls 1) floor2-cases))

(deftest floor3-test
  (run-one-floor (floor-walls 2) floor3-cases))
