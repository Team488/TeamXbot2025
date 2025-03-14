package competition.subsystems.drive.logic;

public record CreeperAlignmentSuggestion(boolean isSuggestionValid, double suggestedPower) {

    public CreeperAlignmentSuggestion() {
        this(false, 0);
    }

    public CreeperAlignmentSuggestion(double suggestedPower) {
        this(true, suggestedPower);
    }
}