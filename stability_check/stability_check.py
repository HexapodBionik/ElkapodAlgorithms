class StabilityCheck:
    def __init__(self, stability_margin: float):
        self.stability_margin = stability_margin
    
    def set_margin(self, new_margin: float) -> None:
        self.stability_margin = new_margin
        
    def get_margin(self) -> float:
        return self.stability_margin
    
    def check_stability(self) -> bool:
        #TODO make a function that checks if the center of mass is within the shape created by foot contact points
        return True