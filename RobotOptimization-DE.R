# Load necessary libraries
library(DEoptim)  # For DE algorithm
library(ggplot2)  # For plotting
library(pracma)   # For PID implementation
library(matlib)   # For matrix operations, useful in MPC
library(quadprog) # For solving quadratic programming problems used in MPC

# Define target positions for the robot
target_x <- 5
target_y <- 5

# Improved DE optimization simulation function
objective_function <- function(params) {
  x <- 0  # initial position x
  y <- 0  # initial position y
  
  u_x <- params[1]  # control input for x
  u_y <- params[2]  # control input for y
  
  time_steps <- 50
  total_time <- 0
  total_energy <- 0
  total_error <- 0
  
  for (i in 1:time_steps) {
    x <- x + u_x
    y <- y + u_y
    
    time_cost <- abs(x - target_x) + abs(y - target_y)
    energy_cost <- abs(u_x) + abs(u_y)
    accuracy_error <- (x - target_x)^2 + (y - target_y)^2
    
    total_time <- total_time + time_cost
    total_energy <- total_energy + energy_cost
    total_error <- total_error + accuracy_error
  }
  
  total_cost <- 0.5 * total_time + 0.3 * total_energy + 0.2 * total_error
  return(total_cost)
}

# DE optimization settings
lower_bound <- c(-0.5, -0.5)
upper_bound <- c(0.5, 0.5)
control_params <- list(NP = 50, itermax = 300, F = 0.9, CR = 0.8)

# Run DE optimization
de_result <- DEoptim(objective_function, lower = lower_bound, upper = upper_bound, DEoptim.control(control_params))
best_params <- de_result$optim$bestmem
de_time <- sum(abs(best_params[1]) * 50)
de_energy <- sum(abs(best_params[2]) * 50)
de_error <- (de_time - target_x)^2 + (de_energy - target_y)^2

# Simulate PID control
simulate_pid <- function(target, kp, ki, kd) {
  current_position <- 0
  error_sum <- 0
  last_error <- 0
  time_steps <- 50
  positions <- numeric(time_steps)
  
  for (i in 1:time_steps) {
    error <- target - current_position
    error_sum <- error_sum + error
    d_error <- error - last_error
    output <- kp * error + ki * error_sum + kd * d_error
    
    current_position <- current_position + output
    positions[i] <- current_position
    last_error <- error
  }
  
  time_taken <- abs(target - current_position)
  energy_used <- sum(abs(positions))
  accuracy_error <- (target - current_position)^2
  return(c(time_taken, energy_used, accuracy_error))
}

# Results for PID
pid_result <- simulate_pid(5, kp = 0.1, ki = 0.05, kd = 0.01)
pid_time <- pid_result[1]
pid_energy <- pid_result[2]
pid_error <- pid_result[3]

# Simulate MPC control
simulate_mpc <- function(target) {
  A <- matrix(c(1, 0, 0, 1), nrow = 2)
  B <- matrix(c(0.1, 0.1), nrow = 2)
  Q <- diag(2)
  R <- matrix(0.1, nrow = 1, ncol = 1)
  x <- c(0, 0)
  
  N <- 10
  time_steps <- 50
  u <- numeric(time_steps)
  
  total_time <- 0
  total_energy <- 0
  total_error <- 0
  
  for (i in 1:time_steps) {
    Dmat <- 2 * (t(B) %*% Q %*% B + R)
    dvec <- 2 * t(x - c(target, target)) %*% Q %*% B
    
    Amat <- diag(length(dvec))
    bvec <- rep(0.1, length(dvec))
    
    res <- solve.QP(Dmat, as.vector(dvec), t(Amat), bvec, meq = 0)
    u[i] <- res$solution[1]
    
    x <- A %*% x + B * u[i]
    
    time_cost <- abs(x[1] - target) + abs(x[2] - target)
    energy_cost <- sum(abs(u[i]))
    accuracy_error <- (x[1] - target)^2 + (x[2] - target)^2
    
    total_time <- total_time + time_cost
    total_energy <- total_energy + energy_cost
    total_error <- total_error + accuracy_error
  }
  
  return(c(total_time, total_energy, total_error))
}

# Run the simulation for MPC
mpc_result <- simulate_mpc(5)
mpc_time <- mpc_result[1]
mpc_energy <- mpc_result[2]
mpc_error <- mpc_result[3]

# Create a data frame to compare the results
results <- data.frame(
  Method = c("PID", "MPC", "DE"),
  Time = c(pid_time, mpc_time, de_time),
  Energy = c(pid_energy, mpc_energy, de_energy),
  Error = c(pid_error, mpc_error, de_error)
)

# Plotting results using ggplot2 for comparison
library(reshape2)

# Reshape data for grouped bar chart
results_melt <- melt(results, id.vars = "Method")

# Create grouped bar chart for Time, Energy, and Error
ggplot(results_melt, aes(x = Method, y = value, fill = variable)) +
  geom_bar(stat = "identity", position = position_dodge(width = 0.8), width = 0.7) +
  labs(title = "Comparison of Control Methods", y = "Value", x = "Method") +
  scale_fill_manual(values = c("Time" = "red", "Energy" = "green", "Error" = "blue"), 
                    name = "Metrics") +
  theme_minimal() +
  theme(axis.text.x = element_text(angle = 45, hjust = 1))

# Create a line plot to see trends over methods
ggplot(results_melt, aes(x = Method, y = value, color = variable, group = variable)) +
  geom_line(size = 1) +
  geom_point(size = 3) +
  labs(title = "Performance Trends of Control Methods", y = "Value", x = "Method") +
  scale_color_manual(values = c("Time" = "red", "Energy" = "green", "Error" = "blue"),
                     name = "Metrics") +
  theme_minimal()
