package competition;

import competition.electrical_contract.UnitTestContract2025;
import competition.injection.components.CompetitionTestComponent;
import competition.injection.components.DaggerCompetitionTestComponent;
import xbot.common.injection.BaseWPITest;

public class BaseCompetitionTest extends BaseWPITest{
    @Override
    protected CompetitionTestComponent createDaggerComponent() {
        return (CompetitionTestComponent)DaggerCompetitionTestComponent
                .builder()
                .electricalContract(new UnitTestContract2025())
                .build();
    }

    @Override
    protected CompetitionTestComponent getInjectorComponent() {
        return (CompetitionTestComponent)super.getInjectorComponent();
    }
}
