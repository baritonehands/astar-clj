(ns baritonehands.astar
  (:require [io.aviso.ansi :as color])
  (:gen-class))

(defn idx->pos
  ([idx] (idx->pos idx 4))
  ([idx size]
   [(mod idx size) (long (/ idx size))]))

(defn ny-distance [left right]
  (let [[x y] (idx->pos left)
        [cx cy] (idx->pos right)]
    (+ (Math/abs ^long (- cy y))
       (Math/abs ^long (- cx x)))))

(def floor-walls
  [{:v [0, 5, 9, 10],
    :h [1, 4, 6, 11]}
   {:v [0, 1, 2, 9, 10, 11],
    :h [4, 7]},
   {:v [3, 5, 6, 7, 11],
    :h [1, 6, 7]}])

(defn blocked? [vertical? wall [tcol trow] [pcol prow]]
  (let [pos (idx->pos wall 3)
        [wcol wrow] (if vertical? pos (reverse pos))]
    (if vertical?
      (and (= trow prow) (= trow wrow) (= (Math/abs ^long (- tcol pcol)) 1) (= (min tcol pcol) wcol))
      (and (= tcol pcol) (= tcol wcol) (= (Math/abs ^long (- trow prow)) 1) (= (min trow prow) wrow)))))

(defn adjacent? [walls t1 t2]
  (let [[tcol trow] (idx->pos t1)
        [pcol prow] (idx->pos t2)
        adj (or
              (and (= trow prow) (= (Math/abs ^long (- tcol pcol)) 1))
              (and (= tcol pcol) (= (Math/abs ^long (- trow prow)) 1)))
        blocked (->> (for [[k xs] walls
                           wall xs]
                       (blocked? (= k :v) wall [tcol trow] [pcol prow]))
                     (some true?))]
    (and adj (not blocked))))

(defn neighbors [walls idx]
  (->> (range 0 16)
       (filterv #(adjacent? walls idx %))))

(defn walk-path [came-from current]
  (loop [current current
         path (list current)]
    (if (contains? came-from current)
      (recur (came-from current) (conj path (came-from current)))
      path)))

(defn shortest-path [walls start end]
  (loop [open-set #{start}
         came-from {}
         g-score {start 0}
         f-score {start (ny-distance start end)}]
    (if-not (seq open-set)
      :error
      (let [current (->> f-score
                         (sort-by val)
                         (filter #(contains? open-set (key %)))
                         (ffirst))]
        (if (= current end)
          (walk-path came-from current)
          (let [inner (fn []
                        (loop [[neighbor & more] (neighbors walls current)
                               os-inner (disj open-set current)
                               cf-inner came-from
                               gs-inner g-score
                               fs-inner f-score]
                          (if-not neighbor
                            [os-inner cf-inner gs-inner fs-inner]
                            (let [g (inc (gs-inner current))]
                              (if (or (not (gs-inner neighbor))
                                      (< g (gs-inner neighbor)))
                                (recur
                                  more
                                  (conj os-inner neighbor)
                                  (assoc cf-inner neighbor current)
                                  (assoc gs-inner neighbor g)
                                  (assoc fs-inner neighbor (+ g (ny-distance neighbor end))))
                                (recur more os-inner cf-inner gs-inner fs-inner))))))
                [next-os next-cf next-gs next-fs] (inner)]
            (recur next-os next-cf next-gs next-fs)))))))

(defn direction [left right]
  (let [[x y] (idx->pos left)
        [cx cy] (idx->pos right)]
    (cond
      (< cx x) \<
      (< x cx) \>
      (< cy y) \^
      (< y cy) \⌄)))

(defn angle [left right]
  (let [[x y] (idx->pos left)
        [cx cy] (idx->pos right)]
    (Math/atan2 (- cx x) (- y cy))))

(defn heuristic [start end]
  (fn [left right]
    (+ (* 10 (ny-distance left right))
       (+ (angle left right) (angle start end)))))

(defn shortest-path-clockwise [walls start end]
  (let [h (heuristic start end)]
    (loop [open-set #{start}
           came-from {}
           g-score {start 0}
           f-score {start (ny-distance start end)}]
      (if-not (seq open-set)
        :error
        (let [current (->> f-score
                           (sort-by val)
                           (filter #(contains? open-set (key %)))
                           (ffirst))]
          (if (= current end)
            (walk-path came-from current)
            (let [inner (fn []
                          (loop [[neighbor & more] (->> (neighbors walls current))
                                                        ;(sort-by (partial heuristic start)))
                                 os-inner (disj open-set current)
                                 cf-inner came-from
                                 gs-inner g-score
                                 fs-inner f-score]
                            (if-not neighbor
                              [os-inner cf-inner gs-inner fs-inner]
                              (let [g (inc (gs-inner current))]
                                (if (or (not (gs-inner neighbor))
                                        (< g (gs-inner neighbor)))
                                  (recur
                                    more
                                    (conj os-inner neighbor)
                                    (assoc cf-inner neighbor current)
                                    (assoc gs-inner neighbor g)
                                    (assoc fs-inner neighbor (+ g (h neighbor end))))
                                  (recur more os-inner cf-inner gs-inner fs-inner))))))
                  [next-os next-cf next-gs next-fs] (inner)]
              (recur next-os next-cf next-gs next-fs))))))))

(defn all-heuristics
  ([start] (all-heuristics start (range 0 16)))
  ([start ends]
   (for [end ends
         :when (not= start end)
         :let [h (heuristic start end)]]
     (for [current (range 0 16)
           :when (and (not= current start)
                      (not= current end))]
       [[start end current] (h current end)]))))

(defn dir->pos [dir]
  (set (map #(idx->pos % 3) dir)))

(defn walls->pos [walls]
  (-> walls
      (update :v dir->pos)
      (update :h dir->pos)))

(defn print-tile [col highlight?]
  (str
    (if highlight?
      (str color/bold-white-font color/black-bg-font))
    (String/format "%2d" (into-array [col]))
    color/reset-font))

(defn print-floor
  ([walls] (print-floor walls #{}))
  ([walls path]
   (let [rows (partition 4 (range 0 16))
         {:keys [v h]} (walls->pos walls)
         path-xy (set (map #(idx->pos % 4) path))]
     (doseq [[y row] (map-indexed vector rows)]
       (print " ")
       (doseq [[x col] (map-indexed vector row)]
         (print (print-tile col (path-xy [x y])))
         (print (if (v [x y]) " | " "   ")))
       (println)
       (print " ")
       (doseq [x (range 0 4)]
         (print (if (h [y x]) "--   " "     ")))
       (println)))))
