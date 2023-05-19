import random

class AntColonyOptimization:
    def __init__(self, distances, n_ants, n_iterations, decay_factor=0.5, alpha=1, beta=1, q0=0.5):
        self.distances = distances
        self.n_ants = n_ants
        self.n_iterations = n_iterations
        self.decay_factor = decay_factor
        self.alpha = alpha
        self.beta = beta
        self.q0 = q0
        self.n_cities = len(distances)
        self.pheromone = [[1 / (self.n_cities * self.n_cities) for _ in range(self.n_cities)] for _ in range(self.n_cities)]
        self.best_path = None
        self.best_distance = float('inf')

    def run(self):
        for _ in range(self.n_iterations):
            paths = self.construct_paths()
            self.update_pheromone(paths)
            self.update_best_path(paths)
            self.pheromone = self.decay_pheromone(self.pheromone)
        return self.best_path, self.best_distance

    def construct_paths(self):
        paths = []
        for _ in range(self.n_ants):
            path = self.construct_path()
            paths.append(path)
        return paths

    def construct_path(self):
        visited = [False] * self.n_cities
        start_city = random.randint(0, self.n_cities - 1)
        current_city = start_city
        path = [current_city]
        visited[current_city] = True
        while False in visited:
            next_city = self.choose_next_city(current_city, visited)
            path.append(next_city)
            current_city = next_city
            visited[current_city] = True

        path.append(start_city)
        return path

    def choose_next_city(self, current_city, visited):
        unvisited_cities = [city for city in range(self.n_cities) if not visited[city]]
        if not unvisited_cities:
            return current_city

        probabilities = []
        for city in unvisited_cities:
            probabilities.append((city, self.pheromone[current_city][city] ** self.alpha * (1 / self.distances[current_city][city]) ** self.beta))

        if random.uniform(0, 1) < self.q0:
            max_probability = max(probabilities, key=lambda x: x[1])
            return max_probability[0]
        else:
            total_probability = sum(p[1] for p in probabilities)
            probabilities = [(p[0], p[1] / total_probability) for p in probabilities]
            probabilities.sort(key=lambda x: x[1], reverse=True)
            random_value = random.uniform(0, 1)
            cumulative_probability = 0
            for city, probability in probabilities:
                cumulative_probability += probability
                if random_value <= cumulative_probability:
                    return city

    def update_pheromone(self, paths):
        delta_pheromone = [[0 for _ in range(self.n_cities)] for _ in range(self.n_cities)]
        for path in paths:
            path_distance = self.calculate_distance(path)
            for i in range(len(path) - 1):
                city1 = path[i]
                city2 = path[i + 1]
                delta_pheromone[city1][city2] += 1 / path_distance

        for i in range(self.n_cities):
            for j in range(self.n_cities):
                self.pheromone[i][j] = (1 - self.decay_factor) * self.pheromone[i][j] + delta_pheromone[i][j]

    def update_best_path(self, paths):
        for path in paths:
            path_distance = self.calculate_distance(path)
            if path_distance < self.best_distance:
                self.best_path = path
                self.best_distance = path_distance

    def decay_pheromone(self, pheromone):
        return [[(1 - self.decay_factor) * p for p in row] for row in pheromone]

    def calculate_distance(self, path):
        distance = 0
        for i in range(len(path) - 1):
            city1 = path[i]
            city2 = path[i + 1]
            distance += self.distances[city1][city2]
        return distance


# Example usage:
distances = [
    [0, 1000, 5000],
    [5000, 0, 1000],
    [1000, 5000, 0]
]

n_ants = 10
n_iterations = 100
aco = AntColonyOptimization(distances, n_ants, n_iterations)
best_path, best_distance = aco.run()
print("Best Path:", best_path)
print("Best Distance:", best_distance)