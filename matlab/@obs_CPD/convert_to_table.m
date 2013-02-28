function T = convert_to_table(CPD, domain, evidence)

sz = CPD.sizes;
ns = zeros(1, max(domain));
ns(domain) = sz;

odom = domain(~isemptycell(evidence(domain))); % All observed nodes.
ps = domain(1 : end - 1);
dps = ps(CPD.dps);:

[m, C, hand] = gaussian_CPD_params_given_dps(CPD, domain, evidence);

ns(odom) = 1;
dpsize = prod(ns(dps));
self = domain(end);
assert(myismember(self, odom));
self_val = evidence{self};

T = zeros(dpsize, 1);
for i = 1 : dpsize
  T(i) = gaussian_prob(self_val{1}, m(:, i), C(:, :, i)) * ...
         hand_prob(self_val{2}, hand(:, :, i));
end
end