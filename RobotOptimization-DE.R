# Load necessary libraries
library(DEoptim)  # For DE algorithm
library(ggplot2)  # For plotting
library(pracma)   # For PID implementation
library(matlib)   # For matrix operations, useful in MPC

# Define target positions for the robot
target_x <- 5
target_y <- 5

# Define the objective function for DE optimization
objective_function <- function(params) {
  x <- params[1]
  y <- params[2]
  
  # Define the cost components
  time_cost <- abs(x - target_x)
  energy_cost <- abs(y - target_y)
  accuracy_error <- (x - target_x)^2 + (y - target_y)^2
  
  # Calculate the total cost
  total_cost <- 0.4 * time_cost + 0.3 * energy_cost + 0.3 * accuracy_error
  return(total_cost)
}

# DE optimization settings
lower_bound <- c(-10, -10)
upper_bound <- c(10, 10)
control_params <- list(NP = 30, itermax = 200, F = 0.8, CR = 0.9)

# Run DE optimization
de_result <- DEoptim(objective_function, lower = lower_bound, upper = upper_bound, DEoptim.control(control_params))
de_time <- 10.8  # Hypothetical time obtained for DE

de_energy <- 9.2  # Hypothetical energy obtained for DE
de_error <- 3.2  # Hypothetical accuracy error obtained for DE

# Simulate PID control
simulate_pid <- function(target, kp, ki, kd) {
  # Parameters
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
  # Placeholder simulation for MPC
  time_taken <- 12.3  # Hypothetical time for MPC
  energy_used <- 10.4  # Hypothetical energy for MPC
  accuracy_error <- 4.7  # Hypothetical accuracy error for MPC
  return(c(time_taken, energy_used, accuracy_error))
}

# Results for MPC
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

# Print the results
print(results)






# Plotting the results using ggplot2
ggplot(results, aes(x = Method)) +
  geom_bar(aes(y = Time, fill = "Time"), stat = "identity", position = "dodge") +
  geom_bar(aes(y = Energy, fill = "Energy"), stat = "identity", position = "dodge") +
  geom_bar(aes(y = Error, fill = "Error"), stat = "identity", position = "dodge") +
  scale_fill_manual(values = c("Time" = "red", "Energy" = "green", "Error" = "blue")) +
  labs(title = "Comparison of Control Methods", y = "Value", x = "Method") +
  theme_minimal()

# Radar chart to visualize the overall performance
library(fmsb)
performance_data <- rbind(
  c(max(results$Time), max(results$Energy), max(results$Error)),
  c(min(results$Time), min(results$Energy), min(results$Error)),
  results[1, 2:4],
  results[2, 2:4],
  results[3, 2:4]
)
colnames(performance_data) <- c("Time", "Energy", "Error")

radarchart(performance_data, axistype = 1,
           pcol = c("red", "green", "blue"), pfcol = c(rgb(1,0,0,0.2), rgb(0,1,0,0.2), rgb(0,0,1,0.2)),
           plwd = 2, cglcol = "grey", cglty = 1, axislabcol = "grey", caxislabels = seq(0, max(performance_data), 5), cglwd = 0.8)
legend(x = "topright", legend = c("PID", "MPC", "DE"), col = c("red", "green", "blue"), lty = 1, bty = "n")
