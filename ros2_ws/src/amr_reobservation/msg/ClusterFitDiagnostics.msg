# One cluster's outcome, including clusters skipped before fitting.
# A successful fit is not necessarily the selected nearest fit.
uint32 cluster_index
uint32 point_count
string reason
bool selected

# RANSAC trials attempted (including degenerate samples), separate from PCA.
uint32 ransac_iterations
# Actual PCA calls attempted, including a failed call.
uint32 refinement_iterations
# Configured maximum PCA calls (refinement_limit startup parameter).
uint32 refinement_limit
bool converged
# Size of the set used in the most recent PCA call; zero if never called.
uint32 fit_inlier_count
# Latest available inlier set size, even when the fit was rejected.
uint32 inlier_count
float64 inlier_ratio
# True only if the most recent PCA call reached inlier reclassification.
bool reclassification_available
# Symmetric difference of set membership, NOT the difference in set sizes.
# Meaningful only when reclassification_available=true.
uint32 inlier_change_count

# True only after convergence and finite span/RMSE computation.
# Diagnostic metrics do not imply an accepted fit; check reason.
bool metrics_available
# Zero/default when metrics_available=false; NOT measured zeros.
float64 observed_length_m
float64 rmse_m
