(ns baritonehands.astar
  (:require [io.aviso.ansi :as color]
            [clojure.set :as set])
  (:gen-class))

(defn idx->pos
  ([idx] (idx->pos idx 4))
  ([idx size]
   [(mod idx size) (long (/ idx size))]))

(defn pos+ [[x y] [cx cy]]
  [(+ x cx) (+ cy y)])

(defn pos- [[x y] [cx cy]]
  [(- x cx) (- cy y)])

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

(defn angle [left right]
  (let [[x y] (idx->pos left)
        [cx cy] (idx->pos right)]
    (Math/atan2 (- cx x) (- cy y))))

(defn heuristic [start end]
  (fn [left right]
    (* (angle left right) (angle start end))))

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
                                    (assoc fs-inner neighbor (h neighbor end)))
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

(defn print-tile [col {:keys [start? end? highlight?]}]
  (str
    (cond
      start? (str color/bold-black-font color/white-bg-font)
      end? (str color/bold-black-font color/yellow-bg-font)
      highlight? (str color/bold-white-font color/black-bg-font))
    (String/format "%2d" (into-array [col]))
    color/reset-font))

(defn print-floor
  ([walls] (print-floor walls []))
  ([walls path]
   (let [rows (partition 4 (range 0 16))
         {:keys [v h]} (walls->pos walls)
         pathv (mapv #(idx->pos % 4) path)
         path-xy (set pathv)]
     (doseq [[y row] (map-indexed vector rows)]
       (print " ")
       (doseq [[x col] (map-indexed vector row)]
         (print (print-tile col {:highlight? (path-xy [x y])
                                 :start?     (= [x y] (first pathv))
                                 :end?       (= [x y] (last pathv))}))
         (print (if (v [x y]) " | " "   ")))
       (println)
       (print " ")
       (doseq [x (range 0 4)]
         (print (if (h [y x]) "--   " "     ")))
       (println)))))

(defn directions [left right]
  (let [[x y] (idx->pos left)
        [cx cy] (idx->pos right)
        [dx dy] (pos- [x y] [cx cy])]
    (cond-> #{}
            (pos? dx) (conj :L)
            (neg? dx) (conj :R)
            (pos? dy) (conj :D)
            (neg? dy) (conj :U))))

(def cw-mappings
  {#{:U :L} {#{:D :L} :D
             #{:L :U} :L
             #{:U :R} :R
             #{:R :D} :D
             #{:U :D} :D
             #{:L :R} :L}
   #{:D :L} {#{:D :L} :D
             #{:L :U} :L
             #{:U :R} :R
             #{:R :D} :D
             #{:U :D} :D
             #{:L :R} :R}
   #{:U :R} {#{:D :L} :L
             #{:L :U} :U
             #{:U :R} :U
             #{:R :D} :R
             #{:U :D} :U
             #{:L :R} :L}
   #{:D :R} {#{:D :L} :L
             #{:L :U} :U
             #{:U :R} :U
             #{:R :D} :R
             #{:U :D} :U
             #{:L :R} :R}
   #{:U}    {#{:D :L} :L
             #{:L :U} :L
             #{:U :R} :U
             #{:R :D} :D
             #{:L :R} :L}
   #{:D}    {#{:D :L} :D
             #{:L :U} :U
             #{:U :R} :R
             #{:R :D} :R
             #{:L :R} :R}
   #{:L}    {#{:D :L} :D
             #{:L :U} :L
             #{:U :R} :R
             #{:R :D} :D
             #{:U :D} :D}
   #{:R}    {#{:D :L} :L
             #{:L :U} :U
             #{:U :R} :U
             #{:R :D} :R
             #{:U :D} :U}})

(defn clockwise [current end opts]
  (let [mapper (juxt (comp first (partial directions current)) identity)
        dir->opt (->> (map mapper opts) (into {}))
        dirs (set (keys dir->opt))
        orientation (directions current end)
        cw-mapping (cw-mappings orientation)]
    (dir->opt (cw-mapping dirs))))

(defn init-available [walls path idx opts]
  (if-not (seq opts)
    (set/difference
      (set (neighbors walls idx))
      (set path))
    opts))

(defn path-split [path1 path2]
  (loop [[left & lmore] path1
         [right & rmore] path2
         last= (first path1)]
    (if (not= left right)
      [last= left right]
      (recur lmore rmore left))))

(defn break-tie [end path1 path2]
  (let [[current left right] (path-split path1 path2)
        dir (clockwise current end [left right])]
    ;(println "Breaking tie" path1 path2)
    (if (= dir left)
      path1
      path2)))

(defn dfs [walls start end]
  (loop [path [start]
         avail {start (set (neighbors walls start))}
         paths []]
    (let [current (last path)
          opts (avail current)]
      (cond
        (nil? current)
        (let [ps (->> paths (sort-by count) (partition-by count) first)]
          (if (= (count ps) 2)
            (let [path (apply break-tie end ps)]
              (println (str color/bold-black-font color/green-bg-font start " -> " end color/reset-font "\n"))
              (print-floor walls path)
              path)
            (first ps)))

        (= current end)
        (recur (pop path)
               (update avail (last (pop path)) disj current)
               (conj paths path))

        (empty? opts)
        (recur (pop path)
               (update avail (last (pop path)) disj current)
               paths)

        :else
        (recur (conj path (first opts))
               (update avail (first opts) (partial init-available walls path (first opts)))
               paths)))))

(defn all-dfs
  ([walls start] (all-dfs walls start (range 0 16)))
  ([walls start ends]
   (for [end ends
         :when (not= start end)]
     [[start end] (dfs walls start end)])))

(defn print-all-dfs
  ([walls start] (print-all-dfs walls start (range 0 16)))
  ([walls start ends]
   (doseq [end ends
           :when (not= start end)]
     (println (str color/bold-black-font color/green-bg-font start " -> " end color/reset-font "\n"))
     (print-floor walls (dfs walls start end)))))
