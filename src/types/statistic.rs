pub struct MjStatistic(crate::bindgen::mjStatistic);

macro_rules! impl_statistic_fields {
    ($($name:ident / $set_name:ident: $T:ty = $description:literal;)*) => {
        impl MjStatistic {
            $(
                #[doc = $description]
                pub fn $name(&self) -> $T {
                    self.0.$name
                }
                #[doc = "set "]
                #[doc = $description]
                pub fn $set_name(&mut self, value: $T) {
                    self.0.$name = value;
                }
            )*
        }
    };
}
impl_statistic_fields! {
    meaninertia / set_meaninertia: f64 = "mean diagonal inertia of bodies";
    meanmass / set_meanmass: f64 = "mean mass of bodies";
    meansize / set_meansize: f64 = "mean size of bodies";
    extent / set_extent: f64 = "spatial extent of model";
    center / set_center: [f64; 3] = "center of model in world coordinates";
}
